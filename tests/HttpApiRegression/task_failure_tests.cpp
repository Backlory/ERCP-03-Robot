#include "RPC/Module.hpp"
#include "Net/server.handler.h"
#include <atomic>
#include <future>
#include <iostream>
#include <sstream>
#include <thread>

namespace {
void Require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

enum class State { Ready, Done };

// 实例化生产任务执行器，执行真实函数任务，验证队列和错误状态；不访问机器人设备。
class TaskModule : public module::Module<State> {
    friend class fsmlib::fsm<State>;
public:
    TaskModule() : Module("task-regression", State::Ready) { Resume<TaskModule>(); }
    ~TaskModule() { Shutdown(); }
    bool Submit(const task::tasks_ptr<> &work) { return PostTask(MakeTask(work)); }
    bool Trigger() { return PostAsyncEvent<TaskModule>(Event{}); }
    bool Stop() { return ExecuteStop([this]() { ++stops; return true; }); }
    bool Motion() { return ExecuteMotion([this]() { ++motions; return true; }); }
    void CheckBatch() { RequireCurrentTask(); }
    std::atomic<int> motions{0};
    std::atomic<int> stops{0};
    std::atomic<int> transitions{0};
    bool transition_fails = false;
private:
    struct Event {};
    void Change(const Event &)
    {
        if (transition_fails) throw std::runtime_error("transition failure");
        ++transitions;
    }
    using transition_table = fsm_table<transition_xe<TaskModule, State::Ready, Event,
        State::Done, &TaskModule::Change>>;
};

void WaitIdle(TaskModule &module)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (module.IsBusy() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    Require(!module.IsBusy(), "executor did not finish within 5 seconds");
}

void VerifyFailure(bool throws)
{
    TaskModule module;
    std::promise<void> entered;
    std::promise<void> release;
    auto ready = entered.get_future();
    auto released = release.get_future();
    std::atomic<int> first{0}, later{0}, queued{0};
    auto failed = std::make_shared<task::SequentialTasks<>>("action");
    failed->emplace("first", [&]() { ++first; return true; });
    failed->emplace("failure", [&]() -> bool {
        entered.set_value();
        if (released.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
            throw std::runtime_error("test coordination timeout");
        if (throws) throw std::runtime_error("failure reason");
        return false;
    });
    failed->emplace("later", [&]() { ++later; return true; });
    Require(module.Submit(failed), "initial action rejected");
    Require(ready.wait_for(std::chrono::seconds(5)) == std::future_status::ready,
            "action did not start");
    auto old = std::make_shared<task::SequentialTasks<>>("queued");
    old->emplace("execute", [&]() { ++queued; return true; });
    Require(module.Submit(old), "queue admission failed");
    release.set_value();
    WaitIdle(module);
    const auto error = module.GetFailure();
    Require(module.GetModuleState() == module::module_state::S_Error, "missing error state");
    Require(error.active && error.time_unix_ns > 0 && error.task_name == "action" &&
            error.failed_step == "action/failure", "failure identity/time missing");
    Require(error.report.at("action/first") == task::passed &&
            error.report.at("action/later") == task::ideal &&
            error.report.at("action/failure") == (throws ? task::errored : task::failed),
            "failure report lost step results");
    Require(!error.reason.empty() && (!throws || error.reason == "failure reason"),
            "failure reason lost");
    // 使用生产 HTTP 序列化函数，经过 JSON 文本往返验证诊断字段和纳秒精度。
    std::ostringstream json;
    MakeModuleFailureJson(error).stringify(json);
    Poco::JSON::Parser parser;
    const auto decoded = parser.parse(json.str()).extract<Poco::JSON::Object::Ptr>();
    Require(decoded->getValue<bool>("active") &&
            decoded->getValue<std::int64_t>("time_unix_ns") == error.time_unix_ns &&
            decoded->getValue<std::string>("task") == error.task_name &&
            decoded->getValue<std::string>("step") == error.failed_step &&
            decoded->getValue<std::string>("reason") == error.reason &&
            decoded->getObject("report")->getValue<int>("action/later") == task::ideal,
            "HTTP failure diagnostics changed during JSON serialization");
    Require(first == 1 && later == 0 && queued == 0, "failure continued or retried work");
    Require(!module.Submit(old) && !module.Trigger() && !module.Resume<TaskModule>(),
            "error admitted motion or transition");
    Require(module.Stop() && module.stops == 1, "explicit stop blocked by error");
    Require(module.GetFailure().time_unix_ns == error.time_unix_ns, "stop cleared error");
    Require(module.Rescue<TaskModule>(true), "explicit clear failed");
    Require(!module.GetFailure().active && module.GetLastError().empty(), "clear kept active error");
    Require(module.motions == 0 && module.stops == 1, "clear executed an action");
    auto next = std::make_shared<task::SequentialTasks<>>("action");
    next->emplace("first", [&]() { ++first; return module.Motion(); });
    next->emplace("failure", []() { return true; });
    next->emplace("later", [&]() { ++later; return true; });
    Require(module.Submit(next), "new explicit action rejected after clear");
    WaitIdle(module);
    Require(first == 2 && later == 1 && queued == 0 && module.motions == 1,
            "fresh action skipped a completed step or revived old queue");
    Require(error.active && error.report.at("action/first") == task::passed,
            "diagnostic snapshot changed after clear");
    Require(module.Trigger(), "new transition rejected");
    WaitIdle(module);
    Require(module.transitions == 1 && module.get_current_state() == State::Done,
            "state transition did not execute");
}

void VerifyCancellation(bool stop)
{
    TaskModule module;
    std::promise<void> entered, release;
    auto ready = entered.get_future();
    auto released = release.get_future();
    auto active = std::make_shared<task::SequentialTasks<>>("active");
    active->emplace("wait", [&]() {
        entered.set_value();
        if (released.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
            throw std::runtime_error("test coordination timeout");
        module.CheckBatch();
        return true;
    });
    active->emplace("motion", [&]() { return module.Motion(); });
    Require(module.Submit(active), "cancellation action rejected");
    Require(ready.wait_for(std::chrono::seconds(5)) == std::future_status::ready,
            "cancellation action did not start");
    auto queued = std::make_shared<task::SequentialTasks<>>("queued");
    queued->emplace("motion", [&]() { return module.Motion(); });
    Require(module.Submit(queued), "cancellation queue rejected");
    Require(stop ? module.Stop() : module.Pause<TaskModule>(), "cancellation failed");
    Require(!module.Rescue<TaskModule>(true), "clear admitted an executing old task");
    release.set_value();
    WaitIdle(module);
    Require(!module.GetFailure().active && module.motions == 0, "cancelled action continued");
    Require(module.Rescue<TaskModule>(true), "clear after cancellation failed");
    auto fresh = std::make_shared<task::SequentialTasks<>>("fresh");
    fresh->emplace("motion", [&]() { return module.Motion(); });
    Require(module.Submit(fresh), "fresh action rejected");
    WaitIdle(module);
    Require(module.motions == 1, "old action revived after clear");
}
}

void RunTaskFailureTests()
{
    VerifyFailure(false);
    VerifyFailure(true);
    VerifyCancellation(false);
    VerifyCancellation(true);
    TaskModule module;
    module.transition_fails = true;
    Require(module.Trigger(), "transition admission failed");
    WaitIdle(module);
    Require(module.GetModuleState() == module::module_state::S_Error &&
            module.GetLastError() == "transition failure" && module.transitions == 0,
            "transition failure did not preserve error");
    std::cout << "PASS: task failures, reports, queue invalidation, stop, clear and fresh actions.\n";
}
