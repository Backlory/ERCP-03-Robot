#pragma once
#include <boost/container_hash/hash.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/serialization/vector.hpp>
#include "helper.hpp"
#include "robot_config.h"
#include "utils.h"

namespace net { namespace ipc {

    using namespace boost::interprocess;

    ///////////////////////////////////////////////////////////////////////////

    static std::string get_prefix()
    {
        static const std::string prefix = "IPC-";
        return prefix;
    }

    static std::string get_mutex_postfix()
    {
        static const std::string prefix = "-mutex";
        return prefix;
    }

    enum class ipc_named_segment {
        Hash,
        Data,
        Time,
        Size,
        Counter,
    };

    static std::string get_segment_name(ipc_named_segment seg)
    {
        static const std::map<ipc_named_segment, std::string> names = {
            { ipc_named_segment::Hash, "hash" },
            { ipc_named_segment::Data, "data" },
            { ipc_named_segment::Time, "time" },
            { ipc_named_segment::Size, "size" },
            { ipc_named_segment::Counter, "count" },
        };
        return names.at(seg);
    }

    ///////////////////////////////////////////////////////////////////////////

    /// <summary>
    /// 用于标准库容器的分配器
    /// </summary>
    template <typename T>
    using ipc_allocator = allocator<T, managed_shared_memory::segment_manager>;

    template <typename T>
    using ipc_vector = boost::interprocess::vector<T, ipc_allocator<T>>;

    template <typename T>
    using ipc_allocators = allocator<ipc_vector<T>, managed_shared_memory::segment_manager>;

    template <typename T>
    using ipc_vectors = boost::interprocess::vector<ipc_vector<T>, ipc_allocators<T>>;

    ///////////////////////////////////////////////////////////////////////////

    namespace _detail {

        /// <summary>
        /// 用于自动删除共享内存
        /// </summary>
        struct ipc_remover {

            ipc_remover(const std::string &name)
                : seg(name)
            {
                shared_memory_object::remove(seg.c_str());
            }

            ~ipc_remover() { shared_memory_object::remove(seg.c_str()); }

        protected:
            const std::string seg;
        };

    } // namespace _detail

    ///////////////////////////////////////////////////////////////////////////

    template <typename Src, typename Pkt>
    class ipc_sender : public _detail::ipc_remover {
    public:
        using type = Src;
        using pack_type = Pkt;
        using direction = _output_net_type;

        ipc_sender(const std::string &name, int verbose, int block = 8);
        ~ipc_sender();

        std::string get_name() const { return name; }
        std::string get_address() const { return name; }
        bool is_running() const { return true; }
        bool encode(const Src &src, Pkt &pkt);

        const int verbose;
        ilsr::mutex_data<Src> frame;

    public:
        size_t get_total_size() const;
        size_t get_free_size() const;

    private:
        using seg = ipc_named_segment;
        using Hash_t = ipc_vector<size_t>;
        using Time_t = ipc_vector<double>;
        using Data_t = ipc_vectors<char>;

        void make_memory(size_t size);

    private:
        const std::string name;
        /// <summary>
        /// 多缓存段，保证同步不撕裂
        /// </summary>
        const int block;
        std::atomic<size_t> counter;

        // TODO: Use guid name
        mutable std::mutex mutex;
        std::shared_ptr<managed_shared_memory> memory;

        std::shared_ptr<boost::thread> m_thread;
        void EncodeWorker();
    };

    ///////////////////////////////////////////////////////////////////////////

    template <typename Src, typename Pkt>
    ipc_sender<Src, Pkt>::ipc_sender(const std::string &name, int verbose, int block)
        : name(name)
        , block(block)
        , _detail::ipc_remover(get_prefix() + name)
        , verbose(verbose)
    {
        m_thread = std::make_shared<boost::thread>(boost::bind(&ipc_sender::EncodeWorker, this));
    }

    template <typename Src, typename Pkt>
    void ipc_sender<Src, Pkt>::make_memory(size_t block_size)
    {
        if (!memory) {
            // 大小: 2 x (块数量 * 块大小) + 额外区域(用于信息变量存储)
            // 预留2倍大小, 注意生成数据时不要超过这个限制
            size_t cap = (block * block_size) * 2.0 + 4096;
            cap = std::max<size_t>(cap, 8 << 10); // 最小 8KB
            cap = std::min<size_t>(cap, 128 << 20); // 最大 128MB
            memory = std::make_shared<managed_shared_memory>(
                open_or_create, (get_prefix() + name).c_str(), cap);
            memory->find_or_construct<size_t>(get_segment_name(seg::Size).c_str())(block);
            memory->find_or_construct<size_t>(get_segment_name(seg::Counter).c_str())(0);
            // memory->find_or_construct<Hash_t>(get_segment_name(seg::Hash).c_str())(
            //    block, 0x00, memory->get_segment_manager());
            memory->find_or_construct<Time_t>(get_segment_name(seg::Time).c_str())(
                block, -1, memory->get_segment_manager());
            auto data = memory->find_or_construct<Data_t>(get_segment_name(seg::Data).c_str())(
                ipc_allocators<char>(memory->get_segment_manager()));
            for (int i = 0; i < block; ++i) {
                data->emplace_back(block_size, 0x00, memory->get_segment_manager());
            }

            ROBOT_INFO(true, fmt::format("Create shared memory `{}`.", name))
            ROBOT_INFO(true,
                fmt::format(
                    "    Free space: {} | {} B", memory->get_size(), memory->get_free_memory()))
        }
    }

    template <typename Src, typename Pkt>
    ipc_sender<Src, Pkt>::~ipc_sender()
    {
        if (m_thread) {
            m_thread->interrupt();
            m_thread->join();
        }
        ROBOT_INFO(true, fmt::format("Release shared memory `{}`.", name))
    }

    template <typename Src, typename Pkt>
    size_t ipc_sender<Src, Pkt>::get_total_size() const
    {
        std::lock_guard<decltype(mutex)> _(mutex);
        return memory ? 0 : memory->get_size();
    }

    template <typename Src, typename Pkt>
    size_t ipc_sender<Src, Pkt>::get_free_size() const
    {
        std::lock_guard<decltype(mutex)> _(mutex);
        return memory ? 0 : memory->get_free_memory();
    }

    template <typename Src, typename Pkt>
    void ipc_sender<Src, Pkt>::EncodeWorker()
    {
        ROBOT_THREADNAME(fmt::format("Enc/{}", name).c_str())

        counter = 0;
        bool error = false;
        double time_dump = -1; // 排除重复数据
        // size_t hash_dump = 0x00;

        Pkt pkt;
        std::vector<char> buffer;
        // boost::hash<decltype(buffer)> hasher;
        typename ilsr::mutex_data<Src>::stamped_data src;

        while (!boost::this_thread::interruption_requested()) {
            try {
                auto t0 = std::chrono ::high_resolution_clock::now();
                if (frame.TryGet(src, 0.2) && time_dump != src.time) {

                    counter++;

                    if (encode(src.data, pkt)) {
                        // 序列化结构体数据到数据流
                        {
                            buffer.resize(0);
                            auto i = boost::iostreams::back_inserter(buffer);
                            boost::iostreams::stream<decltype(i)> os(i);
                            boost::archive::binary_oarchive oa(os);
                            oa << pkt;
                            os.flush();
                        }

                        // size_t nhash = hasher(buffer);

                        // 拷贝到共享内存, 一次最多等待1ms
                        // if (nhash != hash_dump) {
                        std::lock_guard<decltype(mutex)> _(mutex);
                        try {
                            if (!memory) {
                                make_memory(buffer.size());
                            }

                            auto size = memory->find<size_t>( //
                                get_segment_name(seg::Size).c_str());
                            auto count = memory->find<size_t>( //
                                get_segment_name(seg::Counter).c_str());
                            auto times = memory->find<Time_t>( //
                                get_segment_name(seg::Time).c_str());
                            auto buffs = memory->find<Data_t>( //
                                get_segment_name(seg::Data).c_str());

                            if (size.second > 0 && size.first && //
                                count.second > 0 && count.first && //
                                times.second > 0 && times.first && //
                                buffs.second > 0 && buffs.first) {
                                // 增长内存块计数
                                const size_t ic = ((*count.first) + 1) % (*size.first);
                                // 拷贝数据到下一个内存块
                                auto &buff = *((*buffs.first).begin() + ic);
                                buff.reserve(buffer.size());
                                for (int i = 0; i < buffer.size(); ++i) {
                                    buff[i] = buffer[i];
                                }
                                buff.resize(buffer.size());
                                // 更新记录
                                (*times.first)[ic] = src.time;
                                (*count.first) = ic;
                                time_dump = src.time;
                                ROBOT_INFO(verbose > 1,
                                    fmt::format("Save: {:08d}, {:.06f}", (*count.first),
                                        (*times.first)[ic]));
                            }
                            error = false;
                            //}
                            // catch (const bad_alloc e) {
                            //    ROBOT_ERROR(verbose > 0, "[Error] " << e.what());
                            //
                            //    ROBOT_INFO(verbose > 0,
                            //        "It smees memory space is not enough, try to grow ...");
                            //    if (memory->get_size() >= 16 * 1024 * 1024) {
                            //        ROBOT_ERROR(verbose > 0,
                            //            fmt::format("Memory has used {} bytes, it seems need
                            //            some
                            //            "
                            //                        "optimization "
                            //                        "other than growing.",
                            //                memory->get_size()));
                            //
                            //    } else {
                            //        memory->grow((get_prefix() + name).c_str(), 1024 * 1024);
                            //        ROBOT_INFO(true,
                            //            fmt::format("    Free space: {} | {} B",
                            //            memory->get_size(),
                            //                memory->get_free_memory()))
                            //    }
                        } catch (std::exception &e) {
                            ROBOT_ERROR(verbose > 0 && !error, "[ShmemError] " << e.what());
                            error = true;
                        }
                        //}
                    }
                }
            } catch (const std::exception &e) {
                ROBOT_ERROR(verbose > 0, "[Error] " << e.what());
            }
            boost::this_thread::interruption_point();
            ilsr::Time::sleep_for(0.0001);
        }
    }
}} // namespace net::ipc