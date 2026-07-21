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
#include <type_traits>
#include <functional>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/signals2/signal.hpp>
#include <fmt/format.h>
#include <fmt/color.h>

#include "fsm.hpp"
#include "task.hpp"
#include "utils.h"

#if defined(USING_LOG) && USING_LOG
#ifdef USING_LOGURU
#include <loguru/loguru.hpp>
#define MODULE_INFO(cond, x)    VLOG_IF_S(loguru::Verbosity_INFO, cond) << x;
#define MODULE_ERROR(cond, x)   VLOG_IF_S(loguru::Verbosity_ERROR, cond) << x;
#define MODULE_THREADNAME(name) loguru::set_thread_name(name);
#else
#include <iostream>
#define MODULE_INFO(cond, x)    ((cond) ? (std::cout << x << std::endl) : (std::cout));
#define MODULE_ERROR(cond, x)   ((cond) ? (std::cout << x << std::endl) : (std::cout));
#define MODULE_THREADNAME(name) (void)0;
#endif
#else
#define MODULE_INFO(cond, x)    (void)0;
#define MODULE_ERROR(cond, x)   (void)0;
#define MODULE_THREADNAME(name) (void)0;
#endif

namespace module {

    enum class module_state {
        S_Running = 0,
        S_Error = 1,
        S_Suspending = 2,
    };

    static std::string do_report(const task::tasks_ptr<> &task, const task::report_t &rep)
    {
        std::string info;
        for (auto &r : rep) {
            info += fmt::format("{}, {}\n", r.first, r.second);
        }
        if (task) {
            auto err = task->get_error();
            if (err) {
                try {
                    std::rethrow_exception(err);
                } catch (std::exception e) {
                    info = info + e.what() + "\n";
                }
            }
        }
        return info;
    }

    class task_error : public std::exception {
    public:
        task_error(const task::tasks_ptr<> &task, const task::report_t &report)
            : m_task(task)
            , m_report(report)
            , std::exception(("Task runs failed: \n" + do_report(task, report)).c_str())
        {
        }

        const task::tasks_ptr<> m_task;
        const task::report_t m_report;
    };

    using task_t = boost::packaged_task<void>;

    /// <summary>
    /// 基于有限状态机(FSM)的抽象模块类
    /// </summary>
    /// <typeparam name="ErrorCode">错误代码类型</typeparam>
    /// <typeparam name="State">状态类型</typeparam>
    template <typename State, typename ErrorCode, bool Threading>
    class Module : public fsmlib::fsm<State> {
    public:
        using state_t = State;

        template <typename... Items>
        using fsm_table = fsmlib::detail::_transition_table<Items...>;

    public:
        ///< 模块是否处于转移过程中
        bool IsTransition() const
        {
            try {
                if (is_transition) {
                    return true;
                }
                std::lock_guard<decltype(m_future_mutex)> lock(m_future_mutex);
                if (m_future.valid()) {
                    if (m_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                        return true;
                    }
                    m_future.get();
                }
            } catch (...) {
            }
            return false;
        }

        ///< 模块是否处于`运行`状态(没有错误和挂起)
        bool IsRunning() const { return _inner_state == module_state::S_Running; }

        ///< 检查状态之间是否存在通路
        template <typename Object>
        bool IsConnected(State to)
        {
            if (to == get_current_state()) {
                return true;
            }
            return fsmlib::fsm<State>::test_connection<Object>(to);
        }

        ///< 模块是否处于`忙碌`状态(正在运行有效任务)
        virtual bool IsBusy() const { return IsTaskBusy(); }

        module_state GetModuleState() const { return _inner_state; }

        virtual std::string GetStateName(const State &state) const { return ""; }

        const std::string GetName() const
        {
            return m_name.size() > 0 ? m_name : typeid(*this).name();
        }

        const std::string GetLastError()
        {
            try {
                boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
                if (m_error && m_error->except_ptr) {
                    boost::rethrow_exception(m_error->except_ptr);
                }
                return "";
            } catch (const std::exception &e) {
                return e.what();
            }
        }

        const boost::exception_ptr &GetError() const
        {
            boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
            return m_error ? m_error->except_ptr : nullptr;
        }

        const std::string GetStateString(std::string extra = "")
        {
            std::string state_str = GetStateName(this->m_state);
            bool errored;
            {
                boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
                errored = m_error && m_error->except_ptr != nullptr;
            }
            if (errored) {
                std::string error_str = GetLastError();
                error_str = "\n    There is an unsolved error: " + error_str;
                extra += error_str;
            }
            char x;
            switch (_inner_state) {
            case module_state::S_Suspending:
                x = 'S';
                break;
            case module_state::S_Error:
                x = 'E';
                break;
            case module_state::S_Running:
                x = 'R';
                break;
            }
            return fmt::format("[{}|{}]<{}> {}", x, GetName(), state_str, extra);
        }

    public:
        /// <summary>
        /// 通知流程控制器：暂停当前采样流程
        /// </summary>
        /// <returns></returns>
        template <typename Object>
        bool Pause()
        {
            return this->PostAsyncEvent<Object>(transition_pause{});
        }

        /// <summary>
        /// 通知流程控制器：继续当前采样流程
        /// </summary>
        /// <returns></returns>
        template <typename Object>
        bool Resume()
        {
            return this->PostAsyncEvent<Object>(transition_resume{});
        }

        /// <summary>
        /// 通知流程控制器：继续当前采样流程
        /// </summary>
        /// <returns></returns>
        template <typename Object>
        bool Rescue(bool resume)
        {
            return this->PostAsyncEvent<Object>(transition_rescue{ resume });
        }

    protected:
        struct transition_rescue {
            transition_rescue(bool resume)
                : resume(resume)
            {
            }

            ///< If resume module to running state after rescued.
            const bool resume = false;
        };

        struct transition_pause {
        };

        struct transition_resume {
        };

        struct transition_error {

            transition_error(
                double t, boost::exception_ptr &e, boost::shared_ptr<ErrorCode> c = nullptr)
                : time(t)
                , except_ptr(e)
                , code_ptr(c)
            {
            }

            ///< Error arrising time.
            const double time;
            ///< Error infomation when last error arised.
            const boost::exception_ptr except_ptr;
            ///< Error code when last error arised. (used to check error).
            const boost::shared_ptr<ErrorCode> code_ptr;
        };

    protected:
        virtual ErrorCode GetErrorCode(const boost::exception_ptr &_exception) const = 0;

        virtual bool OnError(const transition_error &) { return false; }
        virtual void OnPause(const transition_pause &) {}
        virtual bool OnRescue(const transition_rescue &) { return false; }
        virtual void OnResume(const transition_resume &) {}

        /// <summary>
        /// This function is called periodically, do anything periodically that you need here.
        /// </summary>
        /// <param name="state">当前状态</param>
        virtual void Runner(const State &state) {}

        /// <summary>
        /// 发起异步转移
        /// </summary>
        /// <typeparam name="Derived"></typeparam>
        /// <typeparam name="Event"></typeparam>
        /// <param name="event"></param>
        /// <returns></returns>
        template <typename Derived, typename Event>
        bool PostAsyncEvent(const Event &event)
        {
            try {
                static const std::string name = GetName() + "/T";

                if (IsTaskBusy()) {
                    return false;
                }
                ////std::unique_lock<decltype(m_notify_mutex)> lock(m_notify_mutex);
                if (IsTransition()) {
                    if (m_future.wait_for(std::chrono::milliseconds(5))
                        == std::future_status::timeout) {
                        return false;
                    }
                    m_future.get();
                }

                //// Wait until transition is running.
                // while (IsTransition())
                //    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

                // std::lock_guard<decltype(m_future_mutex)> lock(m_future_mutex);
                // if (m_future.valid()) {
                //    if (m_future.wait_for(std::chrono::milliseconds(5)) ==
                //    std::future_status::timeout) {
                //        return false;
                //    }
                //    m_future.get();
                //}

                // boost::mutex::scoped_lock lock(m_notify_mutex);
                m_future = std::async(std::launch::async, [this, event]() {
                    MODULE_THREADNAME(name.c_str())
                    return PostEvent<Derived>(event);
                }).share();
                //// XXX: some wait for transition is running.
                // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return true;
            } catch (...) {
            }
            return false;
        }

        void ClearFaults()
        {
            boost::lock_guard<decltype(m_error_lock)> lock(m_error_lock);
            m_error.reset();
        }

    private:
        State post_event(const transition_error &event)
        {
            fsmlib::fsm<State>::processing_lock lock(*this);
            if (_inner_state != module_state::S_Error) {
                if (!OnError(event)) {
                    _inner_state = module_state::S_Error;
                }
            }
            return get_current_state();
        }

        template <typename Object>
        State post_event(const transition_rescue &event)
        {
            fsmlib::fsm<State>::processing_lock lock(*this);
            if (_inner_state == module_state::S_Error) {
                if (!OnRescue(event)) {
                    return get_current_state();
                }
                ClearFaults();
                _inner_state = module_state::S_Suspending;
                if (event.resume) {
                    OnResume(transition_resume{});
                    _inner_state = module_state::S_Running;
                    MODULE_INFO(_verbose > 0, (*this).GetStateString("Module rescued and resumed."))
                } else {
                    MODULE_INFO(
                        _verbose > 0, (*this).GetStateString("Module rescued and suspended."))
                }
            }
            return get_current_state();
        }

        template <typename Object>
        State post_event(const transition_pause &event)
        {
            fsmlib::fsm<State>::processing_lock lock(*this);
            if (_inner_state == module_state::S_Running) {
                OnPause(event);
                _inner_state = module_state::S_Suspending;
                MODULE_INFO(_verbose > 0, (*this).GetStateString("Module suspended."))
            }
            return get_current_state();
        }

        template <typename Object>
        State post_event(const transition_resume &event)
        {
            fsmlib::fsm<State>::processing_lock lock(*this);
            if (_inner_state == module_state::S_Suspending) {
                OnResume(event);
                _inner_state = module_state::S_Running;
                MODULE_INFO(_verbose > 0, (*this).GetStateString("Module resumed."))
            }
            return get_current_state();
        }

        template <typename Object, typename Event>
        State post_event(const Event &event)
        {
            return fsmlib::fsm<State>::post_event<Object, Event>(event);
        }

    private:
        template <typename Derived, typename Event>
        State PostEvent(const Event &event)
        {
            //{
            //    std::unique_lock<decltype(m_notify_mutex)> lock(m_notify_mutex);
            //    // Notify async posting to continue.
            assert(!is_transition);
            is_transition = true;
            //    //m_notify.notify_all();
            //}
            try {
                auto old_state = this->get_current_state();
                MODULE_INFO(_verbose > 1,
                    GetStateString(
                        fmt::format("Transition started with [{}]", typeid(event).name())))
                bool not_running = _inner_state != module_state::S_Running;
                // Pre transition event
                OnPreTransition(old_state);
                // NOTE: post_event not assign new_state to `m_state`, so do it manually.
                auto new_state = this->post_event<Derived>(event);
                if (_inner_state != module_state::S_Running) {
                    MODULE_ERROR(_verbose > 1,
                        GetStateString("Cannot transition for module is not running."))
                } else {
                    // Assign new state.
                    this->m_state = new_state;
                    MODULE_INFO(_verbose > 1,
                        GetStateString(fmt::format("Transition finished from [{}] with [{}].",
                            GetStateName(old_state), typeid(event).name())))
                    // Post transition event: before assign new state to avoid runner confliction.
                    if (old_state != new_state || not_running) {
                        OnPostTransition(old_state, new_state);
                    }
                }
                //{
                //    // !!! This must be here.
                //    std::unique_lock<decltype(m_notify_mutex)> lock(m_notify_mutex);
                is_transition = false;
                //}
            } catch (std::exception e) {
                //{
                //    std::unique_lock<decltype(m_notify_mutex)> lock(m_notify_mutex);
                is_transition = false;
                //}
                _Do_Error(e);
            }
            return get_current_state();
        }

    protected:
        /// <summary>
        /// Create runnable task by state.
        /// </summary>
        /// <param name=""></param>
        /// <returns></returns>
        virtual std::function<void()> MakeTask(const state_t &) { return nullptr; }

        /// <summary>
        /// Create runnable task for interruption by state.
        /// </summary>
        /// <param name=""></param>
        /// <returns></returns>
        virtual std::function<void()> OnInterrupt(const state_t &) { return nullptr; }

        /// <summary>
        /// Create runnable task.
        /// </summary>
        /// <param name="task"></param>
        /// <returns></returns>
        std::function<void()> MakeTask(const task::tasks_ptr<> &task)
        {
            if (!task)
                return nullptr;
            return [this, task]() {
                MODULE_INFO(
                    _verbose > 1, "Run task: " << task->get_name() << " : " << task->get_count());
                if (!task->run(this->task_report)) {
                    throw task_error(std::move(task), std::move(this->task_report));
                }
                MODULE_INFO(
                    _verbose > 1, module::do_report(this->m_current_task, this->task_report));
                {
                    std::lock_guard<decltype(m_task_mutex)> lock(m_task_mutex);
                    m_current_task = task;
                }

                m_retry = 0;
                this->task_report.clear();
            };
        }

        const task::tasks_ptr<> GetCurrentTask() const
        {
            std::lock_guard<decltype(m_task_mutex)> lock(m_task_mutex);
            return m_current_task;
        }

        const bool IsTaskBusy() const
        {
            std::lock_guard<decltype(m_task_mutex)> lock(m_task_mutex);
            return m_current_task && m_current_task->is_busy();
        }

        /// <summary>
        /// Clear action report, then will re-run the task next time.
        /// </summary>
        void ClearReport() { this->task_report.clear(); }

        /// <summary>
        /// Post task to server running list.
        /// </summary>
        /// <param name="task"></param>
        /// <returns></returns>
        bool PostTask(const std::function<void()> &task)
        {
            if (task) {
                m_task_server.post(task);
                return true;
            }
            return false;
        }

    protected:
        Module(
            std::string name, State init_state, double period, double guard_period, int verbose = 0)
            : fsmlib::fsm<State>(init_state)
            , m_name(name)
            , _init_state(init_state)
            , _period(period)
            , _guard_period(guard_period > 0 ? guard_period : 1.0)
            , _verbose(verbose)
        {
            // assert(period > 0, "Module period should be positive.");
            if (Threading) {
                MODULE_INFO(true, (*this).GetStateString("Module (threading) constucted."))
            } else {
                MODULE_INFO(true, (*this).GetStateString("Module constucted."))
            }

            // 创建后台任务
            m_def_task = boost::make_shared<boost::asio::io_service::work>(m_task_server);

            // Create background runner.
            m_task_worker = boost::make_shared<boost::thread>(&Module::TaskThread, this);

            m_task_server.post([]() {});
        }

        ~Module()
        {
            m_def_task.reset();
            m_task_server.reset();
            m_task_worker->interrupt();
            m_task_worker->join();
            MODULE_INFO(true, (*this).GetStateString("Module destoryed."))
        }

    private:
        void TaskThread()
        {
#ifdef USING_LOGURU
            loguru::set_thread_name((m_name + "-TaskWorker").c_str());
#endif
            while (!boost::this_thread::interruption_requested()) {
                try {
                    m_task_server.run();
                } catch (std::exception &e) {
                    _Do_Error(e);
                }
            }
        }

    protected:
        void WorkerThread()
        {
#ifdef USING_LOGURU
            loguru::set_thread_name((m_name + "-Worker").c_str());
#endif
            while (!boost::this_thread::interruption_requested()) {
                double t0 = ilsr::Time::wall_time();
                try {
                    if (IsRunning() && !IsTransition()) {
                        Runner(get_current_state());
                    }
                } catch (const std::exception &e) {
                    _Do_Error(e);
                }
                _Do_Dispatch(t0);
            }
        }

        void _Do_Dispatch(const double t0)
        {
            // Thread dispatch: decide how long to sleep before next cycle.
            if (_period > 0) {
                double te = ilsr::Time::wall_time();
                double sleep = std::clamp<double>(_period - (te - t0), 0, _period); // second
                std::this_thread::sleep_for(std::chrono::microseconds(int(sleep * 1000000)));
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
            boost::this_thread::interruption_point();
        }

        void _Do_Error(const std::exception &e, const std::string &extra = "", bool ignore = false)
        {

            if (!ignore && _inner_state != module_state::S_Error) {
                auto _ptr = boost::current_exception();
                auto error = std::make_shared<transition_error>(
                    /* time = */ ilsr::Time::wall_time(),
                    /* except_ptr =*/_ptr,
                    /* code_ptr = */ boost::make_shared<ErrorCode>(GetErrorCode(_ptr)));
                {
                    boost::shared_lock_guard<decltype(m_error_lock)> lock(m_error_lock);
                    // Store current error and state.
                    m_error.swap(error);
                }
                // Call error dealing.
                MODULE_ERROR(true, GetStateString(extra))
                try {
                    this->post_event(*m_error);
                } catch (const std::exception &e) {
                    // Error in this stage will be ignored.
                    _Do_Error(e, "", true);
                }
            } else {
                MODULE_ERROR(
                    true, GetStateString("Module error arrised in the transition to error state."))
            }
        }

    protected:
        boost::shared_ptr<boost::thread> m_worker = nullptr;
        ///< OnPreTransition( old state / now state )
        boost::signals2::signal<void(state_t)> OnPreTransition;
        ///< OnPostTransition( old state, new state)
        boost::signals2::signal<void(state_t, state_t)> OnPostTransition;

        const int _verbose = 0;

    protected:
        std::atomic<size_t> m_retry = { 0 };

    private:
        task::report_t task_report;
        boost::asio::io_service m_task_server;
        boost::shared_ptr<boost::thread> m_task_worker = nullptr;
        boost::shared_ptr<boost::asio::io_service::work> m_def_task;

        mutable std::mutex m_task_mutex;
        task::tasks_ptr<> m_current_task;

    private:
        const State _init_state;
        const std::string m_name;
        const double _period;
        const double _guard_period;
        std::atomic<module_state> _inner_state = module_state::S_Suspending;

        boost::shared_mutex m_error_lock;
        std::shared_ptr<transition_error> m_error = nullptr;

    private:
        std::atomic_bool is_transition = false;
        mutable std::mutex m_future_mutex;
        std::shared_future<State> m_future;
    };

} // namespace module

#ifdef _NO_MIN
#pragma pop_macro("min")
#undef _NO_MIN
#endif

#ifdef _NO_MAX
#pragma pop_macro("max")
#undef _NO_MAX
#endif
