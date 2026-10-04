#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "fsm.hpp"
#include "task.hpp"

namespace module {

enum class module_state { S_Running = 0, S_Error = 1, S_Suspending = 2 };

struct FailureInfo {
    bool active = false;
    std::int64_t time_unix_ns = 0;
    std::string task_name;
    std::string failed_step;
    std::string reason;
    std::map<std::string, int> report;
};

inline std::string do_report(const task::tasks_ptr<> &work, const task::report_t &report)
{
    if (work && work->get_error()) {
        try {
            std::rethrow_exception(work->get_error());
        } catch (const std::exception &e) {
            return e.what();
        } catch (...) {
            return "Task raised a non-standard exception.";
        }
    }
    for (const auto &step : report) {
        if (step.second == task::failed || step.second == task::errored)
            return step.first + ": task returned failure.";
    }
    return "Task failed.";
}

class task_error : public std::runtime_error {
public:
    task_error(const task::tasks_ptr<> &work, const task::report_t &report)
        : std::runtime_error(do_report(work, report)), m_task(work), m_report(report) {}
    const task::tasks_ptr<> m_task;
    const task::report_t m_report;
};

// 所有任务及状态转移共用一个执行线程；运行状态、请求批次和设备写入共用同一把锁。
template <typename State>
class Module : public fsmlib::fsm<State> {
public:
    using state_t = State;
    template <typename... Items> using fsm_table = fsmlib::detail::_transition_table<Items...>;

    bool IsRunning() const { return GetModuleState() == module_state::S_Running; }
    module_state GetModuleState() const
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        return m_state_status;
    }
    bool IsTransition() const
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        return m_transition;
    }
    bool IsTaskBusy() const
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        return m_executing || m_pending != 0;
    }
    virtual bool IsBusy() const { return IsTaskBusy(); }
    template <typename Object> bool IsConnected(State to)
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        return to == this->get_current_state() || this->template test_connection<Object>(to);
    }
    virtual std::string GetStateName(const State &) const { return ""; }
    std::string GetName() const { return m_name; }
    FailureInfo GetFailure() const
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        return m_failure;
    }
    std::string GetLastError() const { return GetFailure().reason; }
    std::string GetStateString(std::string extra = "") const
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        return m_name + " / " + GetStateName(this->get_current_state()) + " / " +
               m_failure.reason + " " + extra;
    }

    /// 暂停使当前批次失效；恢复之后只接受新的请求。
    template <typename Object> bool Pause()
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        InvalidateRequests();
        if (m_state_status != module_state::S_Error)
            m_state_status = module_state::S_Suspending;
        return true;
    }
    template <typename Object> bool Resume()
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        if (m_stopping || m_state_status == module_state::S_Error || m_executing)
            return false;
        m_state_status = module_state::S_Running;
        return true;
    }
    /// 人工清除仅更新软件记录；旧请求永久失效，不调用设备或重放任务。
    template <typename Object> bool Rescue(bool resume)
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        if (m_stopping || m_executing) return false;
        InvalidateRequests();
        m_failure = {};
        m_state_status = resume ? module_state::S_Running : module_state::S_Suspending;
        return true;
    }

protected:
    Module(std::string name, State init_state)
        : fsmlib::fsm<State>(init_state), m_name(std::move(name))
    {
        m_def_task = boost::make_shared<boost::asio::io_service::work>(m_task_server);
        m_task_worker = boost::make_shared<boost::thread>(&Module::TaskThread, this);
    }
    ~Module() { Shutdown(); }

    // 派生类析构时先停止线程，确保回调不会访问已经销毁的派生成员。
    void Shutdown()
    {
        {
            std::lock_guard<std::recursive_mutex> lock(m_gate);
            m_stopping = true;
            InvalidateRequests();
            m_def_task.reset();
            m_task_server.stop();
        }
        if (m_task_worker && m_task_worker->joinable()) m_task_worker->join();
    }

    /// 提交和开始执行分别检查状态及批次；错误、暂停、停止、清除均废弃旧批次。
    bool PostTask(const std::function<void()> &work)
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        return Enqueue(work, false);
    }

    template <typename Derived, typename Event> bool PostAsyncEvent(const Event &event)
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        if (IsTaskBusy() || m_transition) return false;
        return Enqueue([this, event]() { ApplyTaskEvent<Derived>(event); }, true);
    }

    // 任务完成后的状态提交在当前执行线程中完成，仍检查请求批次。
    template <typename Derived, typename Event> bool ApplyTaskEvent(const Event &event)
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        RequireCurrentTask();
        this->m_state = fsmlib::fsm<State>::template post_event<Derived, Event>(event);
        return true;
    }

    void RequireCurrentTask() const
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        if (m_stopping || m_state_status != module_state::S_Running || !m_executing ||
            m_executing_generation != m_generation)
            throw task::cancelled();
    }

    /// 状态检查与单次运动写入在锁内连续执行，阻止失效任务继续发送命令。
    template <typename Action> bool ExecuteMotion(Action action)
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        RequireCurrentTask();
        return action();
    }

    /// 显式停止可以在错误状态执行；保留失败诊断，并使当前批次失效。
    template <typename Action> bool ExecuteStop(Action action)
    {
        std::lock_guard<std::recursive_mutex> lock(m_gate);
        if (m_stopping) return false;
        InvalidateRequests();
        try {
            if (!action()) throw std::runtime_error("Stop operation failed.");
            return true;
        } catch (const std::exception &e) {
            RecordFailure(e, "stop");
        } catch (...) {
            RecordFailure(std::runtime_error("Stop raised a non-standard exception."), "stop");
        }
        return false;
    }

    std::function<void()> MakeTask(const task::tasks_ptr<> &work)
    {
        if (!work) return nullptr;
        return [this, work]() {
            task::report_t report;
            work->report(report);
            work->set_execution_guard([this]() { RequireCurrentTask(); });
            if (!work->run(report)) throw task_error(work, report);
        };
    }

private:
    bool Enqueue(const std::function<void()> &work, bool transition)
    {
        if (!work || m_stopping || m_state_status != module_state::S_Running) return false;
        const auto generation = m_generation;
        m_task_server.post([this, work, generation, transition]() {
            {
                std::lock_guard<std::recursive_mutex> lock(m_gate);
                if (m_stopping || generation != m_generation ||
                    m_state_status != module_state::S_Running) return;
                --m_pending;
                m_executing = true;
                m_executing_generation = generation;
            }
            try {
                RequireCurrentTask();
                work();
            } catch (const task::cancelled &) {
                // 暂停或停止造成的取消只终止本任务。
            } catch (const std::exception &e) {
                std::lock_guard<std::recursive_mutex> lock(m_gate);
                RecordFailure(e, transition ? "transition" : "task");
            } catch (...) {
                std::lock_guard<std::recursive_mutex> lock(m_gate);
                RecordFailure(std::runtime_error("Task raised a non-standard exception."), "task");
            }
            std::lock_guard<std::recursive_mutex> lock(m_gate);
            m_executing = false;
            if (generation == m_generation && transition) m_transition = false;
        });
        ++m_pending;
        if (transition) m_transition = true;
        return true;
    }

    void InvalidateRequests()
    {
        ++m_generation;
        m_pending = 0;
        m_transition = false;
    }

    // 错误处理仅保存首个未清除错误并终止本批次，禁止提交任务或调用设备。
    void RecordFailure(const std::exception &e, const std::string &operation)
    {
        InvalidateRequests();
        m_state_status = module_state::S_Error;
        if (m_failure.active) return;
        m_failure.active = true;
        m_failure.time_unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        m_failure.task_name = operation;
        m_failure.failed_step = operation;
        m_failure.reason = e.what();
        if (const auto *failure = dynamic_cast<const task_error *>(&e)) {
            m_failure.task_name = failure->m_task->get_name();
            for (const auto &step : failure->m_report) {
                m_failure.report.emplace(step.first, step.second);
                if (step.second == task::failed || step.second == task::errored)
                    m_failure.failed_step = step.first;
            }
        }
    }

    void TaskThread() { m_task_server.run(); }

    const std::string m_name;
    mutable std::recursive_mutex m_gate;
    module_state m_state_status = module_state::S_Suspending;
    FailureInfo m_failure;
    std::uint64_t m_generation = 0;
    std::uint64_t m_executing_generation = 0;
    std::size_t m_pending = 0;
    bool m_executing = false;
    bool m_transition = false;
    bool m_stopping = false;
    boost::asio::io_service m_task_server;
    boost::shared_ptr<boost::thread> m_task_worker;
    boost::shared_ptr<boost::asio::io_service::work> m_def_task;
};
} // namespace module
