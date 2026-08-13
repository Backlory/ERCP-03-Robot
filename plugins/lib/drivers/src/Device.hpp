#pragma once

#if defined(_WINDOWS) || defined(WINDOWS) || defined(_WIN32) || defined(WIN32)
#ifdef min
#pragma push_macro("min")
#undef min
#define _NO_MIN
#endif

#ifdef max
#pragma push_macro("max")
#undef max
#define _NO_MAX
#endif
#endif

#include <vector>
#include <cmath>
#include <cstring>
#include <memory>
#include <future>
#include <functional>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/signals2.hpp>
#include <fmt/format.h>
#include <fmt/color.h>
#include <priority_deque.hpp>
#include <tbb/concurrent_queue.h>

#include "utils.h"

namespace device {

#if defined(USING_LOG) && USING_LOG
#ifdef USING_LOGURU
#include <loguru/loguru.hpp>
#define DEVICE_INFO(cond, x) VLOG_IF_S(loguru::Verbosity_INFO, cond) << x;
#define DEVICE_ERROR(cond, x) VLOG_IF_S(loguru::Verbosity_ERROR, cond) << x;
#else
#include <iostream>
#define DEVICE_INFO(cond, x) ((cond) ? (std::cout << x << std::endl) : (std::cout));
#define DEVICE_ERROR(cond, x) ((cond) ? (std::cout << x << std::endl) : (std::cout));
#endif
#else
#define DEVICE_INFO(cond, x) (void)0;
#define DEVICE_ERROR(cond, x) (void)0;
#endif

enum working_mode_t : int { Simplex, HalfDuplex, Duplex };

enum class state_t : int { Offline = 0, Opened = 1, Online = 2 };

// Note: WrBufferSize shall not work if Threading is false.
template <typename ReadPacket,
          typename WritePacket,
          working_mode_t Mode,
          typename ErrorCode,
          ErrorCode Def,
          bool Threading = true,
          size_t RxBufferSize = 512, // 0 = unset buffer size (not recommanded)
          size_t WrBufferSize = RxBufferSize>
class Device {
public:
    typedef enum WrPriority : int {
        NegInfty = INT_MIN,
        Lowest = -10,
        Lower = -5,
        Normal = 0,
        Higher = 5,
        Highest = 10,
        PosInfty = INT_MAX,
    };

    class ReadPacketStamped {
    public:
        size_t index;
        ReadPacket packet;
        double t = -1;
    };

    // Note: bigger priority will be written at first.
    class WritePacketStamped {
    public:
        size_t index;
        WritePacket packet;
        double t = -1;
        int priority = 0;
        void *category = nullptr;
        bool succeed = false;
    };

#pragma region Public

public:
    /**
         * @brief Get the module name, default using class name.
         * @return const std::string
         */
    virtual const std::string GetName() const
    {
        return m_name.size() > 0 ? m_name : typeid(*this).name();
    }

    bool IsOnline() const { return _state == state_t::Online; }

    bool IsOpened() const { return _state != state_t::Offline; }

    const size_t GetReadQueueCapacity() const { return RxBufferSize; }

    const size_t GetWriteQueueCapacity() const { return WrBufferSize; }

    const size_t GetReadQueueSize() const { return m_read_queue->size(); }

    const size_t GetWriteQueueSize() const { return m_write_queue->size(); }

    /**
     * @brief 功能：读取设备最近一次异常并转换为可显示文本。
     * @details 机制：在错误锁内复制异常状态并重新抛出保存的异常；没有异常时返回空字符串。
     */
    const std::string GetLastError()
    {
        // 读取错误锁保护的异常指针并转换为可显示文本；没有异常时返回空字符串。
        try {
            boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
            if (_last_exception) {
                boost::rethrow_exception(_last_exception);
            }
            return "";
        } catch (const std::exception &e) {
            return e.what();
        }
    }

    const boost::exception_ptr &GetError() const
    {
        boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
        return _last_exception;
    }

    /**
     * @brief 功能：组合设备名称、生命周期状态和未解决错误，生成诊断文本。
     * @details 机制：读取状态映射和错误标志，再把调用方附加说明拼接到统一格式中。
     */
    const std::string GetStateString(std::string extra = "")
    {
        // 组合设备名称、生命周期状态和未解决错误，形成日志/UI 使用的稳定诊断文本。
        std::string state_str = GetStateName(_state);
        bool errored;
        {
            boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
            errored = _last_code_ptr != nullptr;
        }
        if (errored) {
            std::string error_str = GetLastError();
            error_str = "\n    There is an unsolved error: " + error_str;
            extra += error_str;
        }
        return fmt::format("[{}]<{}>{}", GetName(), state_str, extra);
    }

    const std::string GetStateName(const state_t &state) const
    {
        static const std::map<state_t, std::string> _mapping{
            {state_t::Offline, "state_t::Offline"},
            {state_t::Opened, "state_t::Opened"},
            {state_t::Online, "state_t::Online"},
        };
        return _mapping.at(state);
    }

#pragma endregion Public

#pragma region OCWR

public:
    /**
         * @brief 功能：打开设备、清空旧缓冲并按工作模式启动读写线程。
         * @details 机制：先调用设备实现的 OnOpen，再切换到 Opened 状态、清理故障和队列，最后创建后台线程。
         */
    virtual bool Open()
    {
        try {
            if (_state != state_t::Offline)
                return true;
            if (OnOpen()) {
                AwakeGuard(false);
                _state = state_t::Opened;
                ClearFaults();
                // Clear buffers
                m_read_queue.clear();
                {
                    boost::lock_guard<decltype(m_write_lock)> lock(m_write_lock);
                    m_write_queue.clear();
                }
                // Start threads
                if (Threading) {
                    _Do_StartThread();
                }
                DEVICE_INFO(_verbose > 1, GetStateString("Device online."))
                return true;
            }
        } catch (const std::exception &e) {
            _Do_Error(e);
        }
        return false;
    }

    /**
         * @brief 功能：停止设备读写线程、清空队列并调用底层关闭钩子。
         * @details 机制：先退出工作线程以阻止新 IO，再清空读写缓存，底层 OnClose 成功后才切换为 Offline。
         */
    virtual bool Close()
    {
        try {
            if (_state == state_t::Offline)
                return true;
            // Close working threads
            QuitThreads();
            // Clear buffers
            m_read_queue.clear();
            {
                boost::lock_guard<decltype(m_write_lock)> lock(m_write_lock);
                m_write_queue.clear();
            }
            // Close
            if (OnClose()) {
                _state = state_t::Offline;
                DEVICE_INFO(_verbose > 0, GetStateString("Device offline."))
                return true;
            }
        } catch (const std::exception &e) {
            _Do_Error(e, false);
        }
        return false;
    }

    /**
         * @brief Write something.
         * @param packet The packet to write.
         * @param priority Packet priority.
         * @param out_index Ouptut index of current packet.
         * @param category Category identifier of current packet.
         * @return true
         * @return false
         * @note 1. This function is called when `Threading` is true, otherwise it is
         * user-defined.
         *       2. Wether succeed or fail, packet and time are copied.
         */
    /**
         * @brief 功能：向设备写队列加入带优先级和类别的命令包。
         * @details 机制：线程模式下按容量保留高优先级/最新包，非线程模式直接调用 OnWrite；调用方可取得单调递增索引。
         */
    virtual bool Write(WritePacket &&packet,
                       int priority = (int)Normal,
                       size_t *out_index = NULL,
                       void *category = nullptr)
    {
        if (_state == state_t::Offline)
            return false;
        if (Threading) {
            double time = ilsr::Time::wall_time();
            WritePacketStamped _in{m_write_counter + 1,
                                   std::move(packet),
                                   time,
                                   priority,
                                   category};
            boost::lock_guard<decltype(m_write_lock)> lock(m_write_lock);
            // Drop oldest lower priority packets to keep buffersize
            m_write_queue.push(_in);
            while (m_write_queue.size() > WrBufferSize) {
                m_write_queue.pop_minimum();
            }
            m_write_counter = _in.index;
            if (out_index)
                *out_index = _in.index;
            return true;
        } else {
            OnWrite(packet);
            return true;
        }
        return false;
    }

    [[deprecated]]
    bool Write(WritePacket packet, int priority = (int)Normal, size_t *out_index = NULL)
    {
        return this->Write(std::move(packet), priority, out_index, nullptr);
    }

    virtual void ClearFaults()
    {
        boost::lock_guard<decltype(m_error_lock)> lock(m_error_lock);
        _last_code_ptr.reset();
    }

protected:
    /**
         * @brief Read from buffer in POP mode.
         * @param packet [Out]
         * @param overtime [In] For checking if packet overtime.
         * @param time [Out]
         * @return true
         * @return false
         * @note 1. This function is called when `Threading` is true, otherwise it is
         * user-defined.
         *       2. Wether succeed or fail, packet and time are copied.
         */
    /**
     * @brief 从读队列取出最早到达的一包数据。
     * @details 设备在线时弹出队首并返回采样时间；当指定超时窗口时，返回值表示该数据是否仍然新鲜。
     */
    bool ReadPop(ReadPacket &packet, double overtime = -1, double *time = NULL)
    {
        // 从读队列弹出最早一包；输出采样时间并按 overtime 判断数据是否仍然新鲜。
        if (_state == state_t::Offline)
            return false;
        const double t = ilsr::Time::wall_time();
        ReadPacketStamped _tmp;
        if (m_read_queue.try_pop(_tmp)) {
            packet = std::move(_tmp.packet);
            if (time) {
                *time = _tmp.t;
            }
            if (overtime > 0) {
                return t - _tmp.t <= overtime;
            }
            return true;
        }
        return false;
    }

    /**
         * @brief Read from buffer in NEWEST mode.
         * @param packet [Out]
         * @param overtime [In] For checking if packet overtime.
         * @param time [Out]
         * @return true
         * @return false
         * @note 1. This function is called when `Threading` is true, otherwise it is
         * user-defined.
         *       2. Wether succeed or fail, packet and time are copied.
         */
    /**
         * @brief 功能：丢弃旧读包并取出当前队列中最新的一份数据。
         * @details 机制：在读锁内持续弹出直到队列为空，再对最后一包执行超时检查；无数据或设备离线时返回失败。
         */
    bool ReadNewest(ReadPacket &packet, double overtime = -1, double *time = NULL)
    {
        if (_state == state_t::Offline)
            return false;
        const double t = ilsr::Time::wall_time();
        ReadPacketStamped _tmp;
        // We lock it for pop all older data before newer is pushing.
        {
            bool any = false;
            boost::lock_guard<decltype(m_read_lock)> lock(m_read_lock);
            while (m_read_queue.try_pop(_tmp)) {
                any = true;
            }
            if (!any)
                return false;
        }
        // Return last packet
        packet = std::move(_tmp.packet);
        if (time) {
            *time = _tmp.t;
        }
        if (overtime > 0) {
            return t - _tmp.t <= overtime;
        }
        return false;
    }

    /**
         * @brief Update guard time, to keep guard succeed.
         */
    virtual double AwakeGuard(bool online = true)
    {
        if (online)
            _state = state_t::Online;
        return (m_guard_time = ilsr::Time::wall_time());
    }

#pragma endregion OCWR

#pragma region InnerImpl
protected:
    virtual bool OnOpen() { throw not_implemented_error("OnOpen"); }

    /**
         * @note This function is called when `Threading` is true, otherwise
         *      it is user-defined.
         */
    virtual bool OnRead(ReadPacket &rx_packet) { throw not_implemented_error("OnRead"); }

    /**
         * @note This function is called when `Threading` is true, otherwise
         *      it is user-defined.
         */
    virtual bool OnWrite(WritePacket &tx_packet) { throw not_implemented_error("OnWrite"); }

    /**
         * @note This function is called when `Threading` is true and used in
         *  WriteAndReadThread() , otherwise it is user-defined.
         *      Return false to cancel followed reading.
         */
    virtual bool BetweenWriteAndRead(bool got, WritePacketStamped &tx_packet, ReadPacket &rx_packet)
    {
        return true;
    }

    virtual void EndOfWriteAndRead(const WritePacketStamped &tx_packet) {}
    /** Triggered when some packet is dropped by queue manager. */
    virtual void OnDrop(const WritePacketStamped &tx_packet) {}

    /**
         * @note Close action will alse be called if OnOpen failed to make sure that
         *      device is in safety.
         */
    virtual bool OnClose() { return false; }

    virtual ErrorCode GetErrorCode(const boost::exception_ptr &_exception) { return _default_code; }

    /**
     * @brief 为读包补充序号和时间戳并放入有限容量的读队列。
     * @details 队列满时淘汰旧包后重试入队，保证后台线程不会因缓存占满而阻塞整个设备循环。
     */
    inline void PushRead(const ReadPacket &packet, double time = -1, size_t alt_counter = 0)
    {
        // 阶段一：生成带序号和时间戳的读包；阶段二：在读锁内入队，满队列时淘汰最旧包。
        time = time < 0 ? ilsr::Time::wall_time() : time;
        ++m_read_counter;
        ReadPacketStamped _tmp{alt_counter > 0 ? (size_t)alt_counter : (size_t)m_read_counter,
                               std::move(packet),
                               time};
        ReadPacketStamped _dump;
        // Push utils it is possible.
        size_t counting = 0;
        boost::lock_guard<decltype(m_read_lock)> lock(m_read_lock);
        while (!m_read_queue.try_push(_tmp)) {
            // Pop to reserve enough space.
            // Generally this process should not oversize.
            assert(counting < 3);
            m_read_queue.try_pop(_dump);
            counting++;
        }
    }

    virtual void OnGuard() {}

#pragma endregion InnerImpl

#pragma region Threading
private:
    /**
         * @brief 功能：周期检查设备保活时间，超时后降级状态并调用 OnGuard。
         * @details 机制：每 100 ms 读取单调时间；在线状态超过 guard_period 未刷新时标记 Opened，捕获钩子异常并进入统一错误处理。
         */
    void GuardThread()
    {
#ifdef USING_LOGURU
        loguru::set_thread_name((m_name + "-G").c_str());
#endif
        while (!boost::this_thread::interruption_requested()) {
            double t = ilsr::Time::wall_time();
            if (_state != state_t::Offline) {
                if (m_guard_time > 0 && t - m_guard_time > _guard_period) {
                    DEVICE_ERROR(_verbose > 0, this->GetStateString("Guard failed!"))
                    _state = state_t::Opened;
                    try {
                        OnGuard();
                    } catch (const std::exception &e) {
                        _Do_Error(e, false);
                    }
                    m_guard_time = t;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            boost::this_thread::interruption_point();
        }
    }

    /**
         * @brief 功能：根据 Simplex/HalfDuplex/Duplex 工作模式创建 IO 线程。
         * @details 机制：Simplex 使用合并读写循环，其余模式分别创建读线程和写线程，非法模式直接抛出异常。
         */
    void _Do_StartThread()
    {
        switch (_mode) {
        case Simplex: {
            m_thxes.push_back(boost::make_shared<boost::thread>(&Device::WriteAndReadThread, this));
        } break;
        case HalfDuplex:
        case Duplex: {
            m_thxes.push_back(boost::make_shared<boost::thread>(&Device::ReadThread, this));
            m_thxes.push_back(boost::make_shared<boost::thread>(&Device::WriteThread, this));
        } break;
        default:
            throw std::exception("Bad working mode for device.");
        }
    }

    /**
     * @brief 功能：按设备周期扣除本轮执行耗时并休眠。
     * @details 机制：正周期使用剩余时间等待，非正周期只让出极短时间，避免线程忙等占满 CPU。
     */
    void _Do_Dispatch(std::chrono::high_resolution_clock::time_point t0)
    {
        // Thread dispatch: decide how long to sleep before next cycle.
        if (_period > 0) {
            std::chrono::high_resolution_clock::time_point te =
                std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(te - t0).count();
            double sleep = std::clamp<double>(_period - dt, 0, _period); // second
            std::this_thread::sleep_for(std::chrono::nanoseconds(int(sleep * 1.0e6)));
        } else {
            std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        }
    }

    /**
         * @brief 功能：统一记录设备异常、更新错误状态并按策略关闭底层设备。
         * @details 机制：保存异常、错误码和发生时间后输出诊断；close 为真时调用 OnClose，否则直接转为 Offline。
         */
    void _Do_Error(const std::exception &e, bool close = true)
    {

        {
            boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
            _last_error_time = ilsr::Time::wall_time();
            _last_exception = boost::current_exception();
            auto ec_ptr = boost::make_shared<ErrorCode>(GetErrorCode(_last_exception));
            _last_code_ptr.swap(ec_ptr);
        }
        DEVICE_ERROR(true, GetStateString())
        if (close) {
            try {
                OnClose();
            } catch (const std::exception &e) {
                _Do_Error(e, false);
            }
        } else {
            _state = state_t::Offline;
            DEVICE_ERROR(true, GetStateString("Device closed."))
        }
    }

    /**
         * @brief 功能：设备独立读线程的周期循环。
         * @details 机制：在线时调用 OnRead，将成功数据放入读队列；每轮统一处理异常、周期等待和中断点。
         */
    /**
     * @brief 运行设备的独立读取线程。
     * @details 每轮调用底层 OnRead，成功数据进入读队列；异常、周期调度和线程中断统一由循环收尾处理。
     */
    void ReadThread()
    {
        // 阶段一：在线时调用底层 OnRead；阶段二：成功包入队；阶段三：统一异常处理、周期等待和中断检查。
#ifdef USING_LOGURU
        loguru::set_thread_name((m_name + "-R").c_str());
#endif
        while (!boost::this_thread::interruption_requested()) {
            std::chrono::high_resolution_clock::time_point t0 =
                std::chrono::high_resolution_clock::now();
            try {
                if (_state != state_t::Offline) {
                    ReadPacket packet;
                    if (OnRead(packet)) {
                        PushRead(std::move(packet));
                    } else {
                        // TODO: Read failed
                    }
                }
            } catch (const std::exception &e) {
                _Do_Error(e);
            }
            _Do_Dispatch(t0);
            boost::this_thread::interruption_point();
        }
    }

private:
    /**
         * @brief 功能：从写队列取出最高优先级的最新命令，并丢弃同类别旧命令。
         * @details 机制：在写锁内弹出最大元素，再扫描同 category 且时间更早的项并调用 OnDrop，避免陈旧命令追赶执行。
         */
    bool TryPop(WritePacketStamped &packet)
    {
        boost::lock_guard<decltype(m_write_lock)> lock(m_write_lock);
        if (m_write_queue.size() > 0) {
            packet = std::move(m_write_queue.maximum());
            m_write_queue.pop_maximum();

            // Remove those are the same category but older
            std::vector<decltype(m_write_queue)::const_iterator> remover;
            for (auto p = m_write_queue.begin(); p != m_write_queue.end(); ++p) {
                if (p->category && p->category == packet.category && p->t < packet.t) {
                    remover.push_back(p);
                }
            }
            for (auto &p : remover) {
                OnDrop(*p);
                m_write_queue.erase(p);
            }
            return true;
        }
        return false;
    }

    /**
         * @brief 功能：设备独立写线程的周期循环。
         * @details 机制：持续取出写队列命令并调用 OnWrite，直到队列为空；异常交给统一错误处理。
         */
    void WriteThread()
    {
#ifdef USING_LOGURU
        loguru::set_thread_name((m_name + "-W").c_str());
#endif
        while (!boost::this_thread::interruption_requested()) {
            std::chrono::high_resolution_clock::time_point t0 =
                std::chrono::high_resolution_clock::now();
            try {
                if (_state != state_t::Offline) {
                    bool _got;
                    WritePacketStamped _tmp;
                    do {
                        if ((_got = TryPop(_tmp)) && !OnWrite(_tmp.packet)) {
                            // TODO: write failed
                        }
                    } while (_got);
                }
            } catch (const std::exception &e) {
                _Do_Error(e);
            }
            _Do_Dispatch(t0);
            boost::this_thread::interruption_point();
        }
    }

    /**
         * @brief 功能：Simplex 模式下协调一次写入、一次读取和本地序号更新。
         * @details 机制：优先发送队列命令，再根据 BetweenWriteAndRead 决定是否读取，成功读包进入读队列并结束本轮回调。
         */
    void WriteAndReadThread()
    {
#ifdef USING_LOGURU
        loguru::set_thread_name((m_name + "-W/R").c_str());
#endif
        while (!boost::this_thread::interruption_requested()) {
            std::chrono::high_resolution_clock::time_point t0 =
                std::chrono::high_resolution_clock::now();
            try {
                if (_state != state_t::Offline) {
                    // Write something
                    bool _got;
                    WritePacketStamped _tmp;
                    ReadPacket packet;
                    if (_got = TryPop(_tmp)) {
                        _tmp.succeed = OnWrite(_tmp.packet);
                    }
                    // Do prepare for read something
                    bool isread = true;
                    // Assign ID for local read/write
                    size_t &idx = _tmp.index;
                    if (!_got) {
                        idx = m_write_counter + 1;
                    }
                    isread = BetweenWriteAndRead(_got, _tmp, packet);
                    // Update counter for local read/write
                    if (!_got && isread) {
                        m_write_counter = idx;
                    }
                    // Read something
                    if (isread) {
                        if (OnRead(packet)) {
                            PushRead(std::move(packet), -1, m_write_counter);
                        } else {
                            // TODO: Read failed
                        }
                    }
                    EndOfWriteAndRead(_tmp);
                }
            } catch (const std::exception &e) {
                _Do_Error(e);
            }
            _Do_Dispatch(t0);
            boost::this_thread::interruption_point();
        }
    }
#pragma endregion Threading

protected:
    /**
         * @brief Construct a new Device object
         * @param period Runner() period, in seconds. Negative mean not waiting,
         *      but will still sleep for `10us` for system thread dispacthing.
         * @note
         *  1.Device is supposed to write as fast as any queue item is pushed.
         *      Therefore, period for writing means waiting when queue is empty
         *      before next queue checking is pushed into queue.
         *  2.Writing thread ALWATS send the newest pack.
         * @param name
         * @param period
         * @param guard_period
         * @param verbose
         */
    /**
         * @brief 功能：初始化设备线程参数、读写队列容量和保活线程。
         * @details 机制：保存周期/模式配置，设置队列容量，先建立保活监视线程；工作 IO 线程延迟到 Open 时创建。
         */
    Device(std::string name, double period, double guard_period, int verbose = 1)
        : m_name(name)
        , _mode(Mode)
        , _verbose(verbose)
        , _period(period)
        , _guard_period(guard_period > 0 ? guard_period : 1.0)
    {

        if (RxBufferSize > 0)
            m_read_queue.set_capacity(RxBufferSize);

        if (Threading) {
            DEVICE_INFO(true, (*this).GetStateString("Device (threading) constructed."))
        } else {
            DEVICE_INFO(true, (*this).GetStateString("Device constructed."))
        }

        // Start guard threading
        AwakeGuard(false);
        m_guard_thread = boost::make_shared<boost::thread>(&Device::GuardThread, this);
    }

    /**
         * @brief 功能：按保活线程—工作线程—底层关闭的顺序销毁设备。
         * @details 机制：逐级中断并 join 所有线程，最后调用 OnClose；析构阶段吞掉异常，避免异常穿出析构函数。
         */
    ~Device()
    {
        try {
            // Close guard threading
            QuitGuardThread();
            // Close working threads
            QuitThreads();
            // NOTE: parent deconstructor is called after child's, then
            // `OnClose` should be invalid called.
            OnClose();
        } catch (...) {
        }
        DEVICE_INFO(true, (*this).GetStateString("Device destoryed."))
    }

    void QuitGuardThread()
    {
        // Close guard threading
        if (m_guard_thread) {
            m_guard_thread->interrupt();
            m_guard_thread->join();
            m_guard_thread.reset();
        }
    }

    /**
     * @brief 功能：中断并等待全部设备工作线程退出。
     * @details 机制：先统一发出 interruption，再逐个 join，最后清空线程句柄列表，确保底层资源销毁前没有后台访问。
     */
    void QuitThreads()
    {
        // Close working threads
        for (auto &thx : m_thxes) {
            thx->interrupt();
        }
        for (auto &thx : m_thxes) {
            // thx->try_join_for(
            //    boost::chrono::milliseconds(_period > 0 ? int(_period * 1000) : 500));
            thx->join();
        }
        m_thxes.clear();
    }

protected:
    const std::string m_name;
    boost::atomic<int> _verbose = 0;

    ///< Device thread for periodical tasks.
    std::vector<boost::shared_ptr<boost::thread>> m_thxes;

    ///< Read-Writer locker mutex
    boost::shared_mutex m_read_lock;
    boost::shared_mutex m_write_lock;
    boost::shared_mutex m_error_lock;

    ///< User-defined error code.
    const ErrorCode _default_code = Def;
    ///< Error code when last error arised. (used to check error)
    boost::shared_ptr<ErrorCode> _last_code_ptr = nullptr;
    ///< Error infomation when last error arised
    boost::exception_ptr _last_exception;
    boost::atomic<double> _last_error_time = -1;

protected:
    struct greater {
        constexpr bool operator()(const WritePacketStamped &a, const WritePacketStamped &b) const
        {
            // Sort rule:
            //  1. priority is higher
            //  2. time is newer
            if (a.priority == b.priority) {
                return a.t < b.t;
            }
            return a.priority < b.priority;
        }
    };

protected:
    boost::atomic<size_t> m_write_counter = 0;

private:
    boost::atomic<size_t> m_read_counter = 0;
    const double _period;
    const double _guard_period;
    const working_mode_t _mode;

private:
    boost::atomic<state_t> _state = {state_t::Offline};

    boost::container::priority_deque< //
        WritePacketStamped,
        std::vector<WritePacketStamped>,
        greater>
        m_write_queue;
    tbb::concurrent_bounded_queue<ReadPacketStamped> m_read_queue;

    ///< Device guard manager.
    boost::shared_ptr<boost::thread> m_guard_thread;
    boost::atomic<double> m_guard_time = {-1};
};

} // namespace device

#ifdef _NO_MIN
#pragma pop_macro("min")
#undef _NO_MIN
#endif

#ifdef _NO_MAX
#pragma pop_macro("max")
#undef _NO_MAX
#endif
