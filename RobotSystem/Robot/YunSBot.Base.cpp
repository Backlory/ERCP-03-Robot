#include <assert.h>
#include <cmath>
#include <vector>
#include <boost/make_shared.hpp>
#include "task.hpp"
#include "robot_settings.hpp"
#include "robot_devices.h"
#include "robot_config.h"
#include "YunSBot.h"
#include "control_cycle_policy.hpp"
#include "RPC/ArmModule.hpp"

using namespace task;

namespace ercp {

#define CMD_BACK (11)
#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

YunSBot::YunSBot()
    : base(*this)
    , master(*this, protocol::v3::Source::Master, GetSettings().Basic.Master(), 31001, 31002, false)
    , situaware(*this, protocol::v3::Source::Cloud, "127.0.0.1", 31003, 31004, true)
{
    ROBOT_INFO(true, "Start lingcai robot !");
    base.StartThreads();

    rpc::ArmModule::GetInstance().Resume<rpc::ArmModule>();
}

/**
 * @brief 初始化 RobotSystem 基类的控制通道、生命周期任务和周期回调。
 * @details 建立状态发送/命令接收通道，登记启动、停止和错误相关事件，并读取 Master 优先仲裁配置。
 */
YunSBot::_base::_base(YunSBot &p)
    : m_status_session_id(robot_udp_v3::MakeSessionId())
    , parent(p)
{
    m_lifecycle_changed_unix_ns = robot_udp_v3::UnixNowNs();
    // Task 11: Master 强制优先仲裁开关,默认从配置读(basic.master_priority,默认 true)。
    m_master_priority = GetSettings().Basic.MasterPriority();
    InitStartingTask();
    InitClosingTask();
    InitBackgroundTask();

    BeforeRobotStarting.connect([&]() {
        m_RobotAutoMode = false;
        m_active_source = 0;
        m_command_fresh = false;
        m_accepted_command_received_unix_ns = 0;
        m_lifecycle_changed_unix_ns = robot_udp_v3::UnixNowNs();
    });
    OnRobotStartSucceed.connect([&]() {
        m_lifecycle_changed_unix_ns = robot_udp_v3::UnixNowNs();
        StartControlThreads();
    });
    OnRobotStartFailed.connect([&]() { m_lifecycle_changed_unix_ns = robot_udp_v3::UnixNowNs(); });
    BeforeRobotStopping.connect([&]() {
        const bool automaticMode = m_RobotAutoMode.load();
        const auto selectedSource = robot_udp_v3::SelectedControlSource(automaticMode);
        m_RobotAutoMode = false;
        ExitControlThreads();

        // The control loop is gone, so explicitly make the final PLC write a safety zero.
        // This uses the unchanged Beckhoff native 10-double + 6-BOOL command layout.
        protocol::v3::ControlPayload ignored;
        robot_udp_v3::CommandMetadata metadata;
        auto &channel = automaticMode ? parent.situaware : parent.master;
        if (!channel.LatestCommand(ignored, metadata)) {
            metadata.source = selectedSource;
        }
        const auto zero = robot_udp_v3::ZeroControl();
        const auto followCommand = control_cycle::ToFollowCommand(zero);
        const auto appliedAt = robot_udp_v3::UnixNowNs();
        const auto adsError = GetRobot().BeckhoffWriteFollowCommand(followCommand);
        const bool succeeded = adsError == 0;
        m_applied_commands.MarkAttempt(zero,
                                       metadata,
                                       succeeded ? protocol::v3::ApplyResult::TimedOutToZero
                                                 : protocol::v3::ApplyResult::Failed,
                                       adsError,
                                       appliedAt,
                                       succeeded);

        m_active_source = 0;
        m_command_fresh = false;
        m_accepted_command_received_unix_ns = 0;
        m_lifecycle_changed_unix_ns = robot_udp_v3::UnixNowNs();
    });
    OnRobotStopFailed.connect([&]() { StartControlThreads(); });
    OnRobotStopEnd.connect([&]() { m_lifecycle_changed_unix_ns = robot_udp_v3::UnixNowNs(); });

    work = boost::make_shared<boost::asio::io_service::work>(this->io_service);
}

/**
 * @brief 功能：启动机器人后台周期线程和 Asio IO 线程。
 * @details 机制：后台线程运行通用周期循环，IO 线程运行 io_service；两者异常分别记录，不改变既有调度周期。
 */
void YunSBot::_base::StartThreads()
{
    m_bg_worker = boost::make_shared<boost::thread>(
        [this]() { RunPeriodicLoop("BgAuto", 0.020, OnBackground); });

    worker = boost::make_shared<std::thread>([this]() {
        try {
            boost::system::error_code ec;
            io_service.run(ec);
        } catch (std::exception e) {
            ROBOT_ERROR(true, "IO context: " << e.what())
        }
    });
}

/**
 * @brief 功能：在机器人启动成功后创建控制周期线程。
 * @details 机制：只在控制线程不存在时创建，避免重复启动；控制回调由统一周期循环驱动。
 */
void YunSBot::_base::StartControlThreads()
{
    if (!m_ctrl_worker) {
        m_ctrl_worker = boost::make_shared<boost::thread>(
            [this]() { RunPeriodicLoop("BgAuto", 0.008, OnControl); });
    }
}

/**
 * @brief 功能：以固定周期执行一个 signals2 回调，直到线程被中断。
 * @details 机制：记录周期起点、隔离回调异常、扣除执行耗时后休眠，并在每轮末检查中断点。
 */
void YunSBot::_base::RunPeriodicLoop(const char *threadName,
                                     double intervalSeconds,
                                     boost::signals2::signal<void(double)> &callback)
{
    ROBOT_THREADNAME(threadName);

    while (!boost::this_thread::interruption_requested()) {
        const double cycleStarted = ilsr::Time::wall_time();
        try {
            callback(cycleStarted);
        } catch (const std::exception &) {
            // Periodic callbacks historically isolate failures and continue the next cycle.
        }

        const double elapsed = ilsr::Time::wall_time() - cycleStarted;
        ilsr::Time::sleep_for(std::max(0.0, intervalSeconds - elapsed));
        boost::this_thread::interruption_point();
    }
}

void YunSBot::_base::ExitControlThreads()
{
    if (m_ctrl_worker) {
        m_ctrl_worker->interrupt();
        m_ctrl_worker->join();
        m_ctrl_worker.reset();
    }
}

/**
 * @brief 关闭 RobotSystem 基类持有的 IO、后台线程和未完成生命周期任务。
 * @details 先停止 Asio 工作，再中断后台线程；若机器人仍在运行则排队停止并等待其 future 完成。
 */
YunSBot::_base::~_base()

{
    work.reset();
    io_service.reset();

    // Stop worker
    if (m_bg_worker) {
        m_bg_worker->interrupt();
        m_bg_worker->join();
    }
    if (m_RobotStarted) {
        Stop();
        m_future.wait();
    }
}

/**
 * @brief 功能：启动异步机器人开机任务，并拒绝与上一生命周期任务重叠。
 * @details 机制：先等待已有 future 完成，再创建 ExecuteStart 异步任务；调用方通过返回值知道任务是否已排入。
 */
bool YunSBot::_base::Start(size_t total)
{
    if (!WaitForLifecycleTask())
        return false;

    m_future = std::async(std::launch::async, [this, total]() { return ExecuteStart(total); });
    return true;
}

bool YunSBot::_base::WaitForLifecycleTask()
{
    if (!m_future.valid())
        return true;
    if (m_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout)
        return false;

    m_future.get();
    return true;
}

/**
 * @brief 按指定次数执行开机任务序列并记录每次失败。
 * @details 每次失败都输出任务错误并递减剩余次数，任意一次成功立即结束；耗尽次数后返回失败。
 */
bool YunSBot::_base::RunStartTasks(std::size_t totalAttempts)
{
    std::size_t remainingAttempts = totalAttempts;
    while (remainingAttempts > 0) {
        ROBOT_INFO(true,
                   fmt::format("Robot start try {} ...", totalAttempts - remainingAttempts))
        if (InitTasks->run(m_init_report))
            return true;

        LogTaskError(InitTasks->get_error());
        --remainingAttempts;
    }
    return false;
}

/**
 * @brief 功能：执行一次完整开机生命周期，包括前置事件、重试任务和成功/失败事件。
 * @details 机制：先拒绝已运行状态，再设置 Starting、准备报告并执行任务；失败保持未启动，成功启动控制线程并清理旧关机报告。
 */
bool YunSBot::_base::ExecuteStart(std::size_t totalAttempts)
{
    if (IsRobotRunning())
        return false;

    ROBOT_INFO(true, "Robot starting ...")
    m_RobotStarting = true;
    if (m_init_report.empty())
        InitTasks->report(m_init_report);
    BeforeRobotStarting();

    if (!RunStartTasks(totalAttempts)) {
        ROBOT_INFO(true, "Robot start failed.")
        ROBOT_INFO(true, GetStartInfo());
        OnRobotStartFailed();
        m_RobotStarting = false;
        return false;
    }

    ROBOT_INFO(true, "Robot started.")
    ROBOT_INFO(true, GetStartInfo());
    m_RobotStarted = true;
    OnRobotStartSucceed();
    m_RobotStarting = false;
    m_deinit_report.clear();
    return true;
}

/**
 * @brief 功能：校验机械臂可关机条件并排入异步关机任务。
 * @details 机制：折叠状态不满足时立即拒绝；其余情况等待已有任务后创建 ExecuteStop，实际停止流程由异步任务完成。
 */
bool YunSBot::_base::Stop()
{
    // 需等到机械臂折叠后才能关机
    auto &arm = rpc::ArmModule::GetInstance();
    if (arm.get_current_state() > rpc::arm_state_t::A3_Folded) {
        return false;
    }

    if (!WaitForLifecycleTask())
        return false;

    m_future = std::async(std::launch::async, [this]() { return ExecuteStop(); });
    return true;
}

/**
 * @brief 功能：执行关机任务、停止控制线程并发布生命周期结果。
 * @details 机制：通过任务序列完成设备收拢，成功后切换未启动状态；失败保留启动状态并触发失败事件供控制线程恢复。
 */
bool YunSBot::_base::ExecuteStop()
{
    if (!IsRobotRunning())
        return false;

    ROBOT_INFO(true, "Robot stopping ...")
    m_RobotStopping = true;
    BeforeRobotStopping();
    if (m_deinit_report.empty())
        DeinitTasks->report(m_deinit_report);

    if (!DeinitTasks->run(m_deinit_report)) {
        ROBOT_INFO(true, "Robot stop failed.")
        ROBOT_INFO(true, GetStopInfo());
        OnRobotStopFailed();
        OnRobotStopEnd();
        m_RobotStopping = false;
        return false;
    }

    // Keep the legacy completion-error source unchanged for compatibility.
    LogTaskError(InitTasks->get_error());
    ROBOT_INFO(true, "Robot stopped.")
    ROBOT_INFO(true, GetStopInfo());
    m_RobotStarted = false;
    if (OnRobotStopped)
        OnRobotStopped();
    OnRobotStopEnd();
    m_RobotStopping = false;
    m_init_report.clear();
    return true;
}

void YunSBot::_base::LogTaskError(const std::exception_ptr &error) const
{
    try {
        std::rethrow_exception(error);
    } catch (const std::exception &exception) {
        ROBOT_ERROR(true, "Error: " << exception.what())
    }
}

std::vector<std::pair<std::string, int>> YunSBot::_base::GetStartReport() const
{
    std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
    std::vector<std::pair<std::string, int>> map;
    for (auto p : m_init_report) {
        map.emplace_back(p);
    }
    return map;
}

std::vector<std::pair<std::string, int>> YunSBot::_base::GetStopReport() const
{
    std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
    std::vector<std::pair<std::string, int>> map;
    for (auto p : m_deinit_report) {
        map.emplace_back(p);
    }
    return map;
}

std::vector<std::pair<std::wstring, int>> YunSBot::_base::GetStartReportW() const
{
    std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
    std::vector<std::pair<std::wstring, int>> map;
    for (auto p : m_init_report) {
        map.emplace_back(ilsr::convert(p.first), p.second);
    }
    return map;
}

std::vector<std::pair<std::wstring, int>> YunSBot::_base::GetStopReportW() const
{
    std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
    std::vector<std::pair<std::wstring, int>> map;
    for (auto p : m_deinit_report) {
        map.emplace_back(ilsr::convert(p.first), p.second);
    }
    return map;
}

std::string YunSBot::_base::GetStartInfo() const
{
    std::string info = "Robot start report:\r\n";
    auto rep = GetStartReport();
    for (auto &r : rep) {
        info += fmt::format("{}, {}\r\n", r.first, r.second);
    }
    return std::move(info);
}

std::string YunSBot::_base::GetStopInfo() const
{
    std::string info = "Robot stop report:\r\n";
    auto rep = GetStopReport();
    for (auto &r : rep) {
        info += fmt::format("{}, {}\r\n", r.first, r.second);
    }
    return std::move(info);
}

bool YunSBot::_base::IsAutoMode() const
{
    return m_RobotAutoMode;
}

bool YunSBot::_base::IsRobotRunning() const
{
    return m_RobotStarted;
}

bool YunSBot::_base::IsRobotStarting() const
{
    return m_RobotStarting;
}

bool YunSBot::_base::IsRobotStopping() const
{
    return m_RobotStopping;
}

bool YunSBot::_base::IsLogging() const
{
    std::lock_guard<decltype(m_log_mutex)> _(m_log_mutex);
    return m_logger != nullptr;
}

boost::asio::io_service &YunSBot::_base::GetIOServer()
{
    return io_service;
}

bool YunSBot::_base::SwitchAutoMode(bool enable)
{
    const bool autoSourceOnline = parent.situaware.IsOnline(0.1);
    if (enable && IsRobotRunning() && autoSourceOnline) {
        m_RobotAutoMode = true;
    } else {
        m_RobotAutoMode = false;
    }
    return true;
}

/**
 * @brief 开启或关闭机器人运行日志文件。
 * @details 在日志锁内按需创建带表头的 CSV logger 或释放现有 logger，返回最终是否处于记录状态。
 */
bool YunSBot::_base::SwitchLogger(bool enable)
{
    std::lock_guard<decltype(m_log_mutex)> _(m_log_mutex);
    if (enable) {
        if (!m_logger) {
            m_logger = std::make_shared<ilsr::Logger>(ilsr::Time::logtime() + ".csv", GetLogPath());
            m_logger->AddLog("t,bx,by,bz,br,x,y,z,r,vx,vy,vz,vr,sf,sq,cf,wf,hf\n");
        }
    } else {
        if (m_logger) {
            m_logger.reset();
        }
    }
    return m_logger != nullptr;
}

/**
 * @brief 将一次力和轴位置采样追加到运行日志。
 * @details 仅在 logger 已启用时按固定 CSV 列顺序格式化数据，并在日志锁内写入一行。
 */
bool YunSBot::_base::AddFLog(double dF1,
                             double dF2,
                             double dF3,
                             double dL,
                             double dAx1,
                             double dAx2,
                             double dAx3)
{
    if (m_logger) {
        char buf[1024];
        sprintf(buf,
                "'%s,%f,%f,%f,%f,%f,%f,%f\n",
                ilsr::Time::timestamp().c_str(),
                dL,
                dF1,
                dF2,
                dF3,
                dAx1,
                dAx2,
                dAx3);
        std::lock_guard<decltype(m_log_mutex)> _(m_log_mutex);

        m_logger->AddLog(buf);
    }
    return true;
}

bool YunSBot::_base::AddFRecord(double dFValue, double dPos)
{
    if (!m_FRecord) {
        m_FRecord = std::make_shared<ilsr::Logger>("Force_Record.csv", "d:\\cyl\\record\\");
        m_FRecord->AddLog("\n\n");
    }

    m_FRecord->AddLog(std::to_string(dFValue) + "," + std::to_string(dPos) + "\n");

    return true;
}

///////////////////////////////////////////////////////////////////////////

/**
 * @brief 构造开机任务序列。
 * @details 先初始化机械臂模块，再依据 Beckhoff 当前臂状态同步 ArmModule 状态机，供 Start 生命周期重试执行。
 */
void YunSBot::_base::InitStartingTask()
{
    InitTasks = std::make_shared<SequentialTasks<>>(u8"开机");
    using task = _TaskBase<>;

    InitTasks->emplace(
        u8"启动机械臂模块",
        []() {
            auto &arm = rpc::ArmModule::GetInstance();
            arm.Initialize();
            sleep_ms(200);
            return arm.get_current_state() >= rpc::ArmModule::state_t::A2_Inited;
        },
        false,
        1);

    InitTasks->emplace(
        u8"更新机械臂状态",
        []() {
            auto &arm = rpc::ArmModule::GetInstance();
            auto &robot = GetRobot();

            if (beckhoff_arm_move_state::BAMS_OPENED == robot.BeckhoffArmMoveState()) {
                return arm.GotoState(rpc::arm_state_t::A4_Opened);
            } else if (beckhoff_arm_move_state::BAMS_FOLDED == robot.BeckhoffArmMoveState()) {
                return arm.GotoState(rpc::arm_state_t::A3_Folded);
            } else if (beckhoff_arm_move_state::BAMS_FOLLOWING == robot.BeckhoffArmMoveState() ||
                       beckhoff_arm_move_state::BAMS_FOLLOWED == robot.BeckhoffArmMoveState()) {
                return arm.GotoState(rpc::arm_state_t::A5_Following);
            }
            return true;
        },
        false,
        1);

    ROBOT_INFO(true, "Robot starting list:\n" << InitTasks->dump());
}

/**
 * @brief 构造关机任务序列。
 * @details 注册机械臂模块反初始化和并行关闭任务，供 Stop 生命周期统一执行。
 */
void YunSBot::_base::InitClosingTask()
{
    DeinitTasks = std::make_shared<SequentialTasks<>>(u8"关机");

    // 注册关闭行为
    {
        auto paral = std::make_shared<ParallelTasks<>>(u8"关闭电机");

        DeinitTasks->emplace(
            u8"关闭机械臂模块",
            []() {
                auto &arm = rpc::ArmModule::GetInstance();
                return arm.DeInitialize();
            },
            true,
            1);

        DeinitTasks->emplace(paral);
    }

    //ROBOT_INFO(true, "Robot closing list:\n" << DeinitTasks->dump());
}

/**
 * @brief 注册后台状态发布和控制周期回调。
 * @details 后台回调按约 20 ms 发布新状态包；控制回调连接到固定周期的 ControlRunnable2。
 */
void YunSBot::_base::InitBackgroundTask()
{
    OnBackground.connect([this](double t) {
        static double t0 = 0;
        if (t - t0 > 0.02) {
            const auto packet = BuildStatusPacket();
            if (!packet.empty()) {
                parent.master.SendStatus(packet);
                parent.situaware.SendStatus(packet);
            }
            t0 = t;
        }
    });

    // 主从控制线程
    OnControl.connect(
        boost::bind(&YunSBot::_base::ControlRunnable2, this, boost::placeholders::_1));
}

protocol::v3::AppliedCommandPayload YunSBot::_base::AppliedCommands() const
{
    return m_applied_commands.Snapshot();
}

protocol::v3::Source YunSBot::_base::ActiveSource() const
{
    return static_cast<protocol::v3::Source>(m_active_source.load());
}

std::uint64_t YunSBot::_base::AcceptedCommandReceivedUnixNs() const
{
    return m_accepted_command_received_unix_ns.load();
}

std::uint64_t YunSBot::_base::LifecycleChangedUnixNs() const
{
    return m_lifecycle_changed_unix_ns.load();
}

/**
 * @brief 功能：从当前生命周期、Beckhoff 快照和命令审计记录构造 Robot V3 状态包。
 * @details 机制：先复制稳定状态快照并映射枚举/位域，再填充采样时间、ADS 诊断和扩展组，最后通过共享编码器生成 wire 包。
 */
protocol::v3::Bytes YunSBot::_base::BuildStatusPacket()
{
    // 阶段一：读取最新 Beckhoff 快照，只对新的、有效的 common 采样生成状态包。
    const auto snapshot = GetRobot().BeckhoffSnapshot();
    const auto common_sample_unix_ns = snapshot.sampled_at_unix_ns[0];
    const bool has_fresh_common_sample =
        (snapshot.valid_groups & device::beckhoff::SnapshotCommon) != 0 &&
        (snapshot.stale_groups & device::beckhoff::SnapshotCommon) == 0 &&
        common_sample_unix_ns != 0;
    if (!has_fresh_common_sample || common_sample_unix_ns == m_last_sent_common_sample_unix_ns) {
        return {};
    }

    // 阶段二：把生命周期、Beckhoff、ERCP 和应用命令审计信息映射到协议载荷。
    const auto now = robot_udp_v3::UnixNowNs();
    protocol::v3::FullStatusPayload status;

    if (IsRobotStopping())
        status.runtime.lifecycle = protocol::v3::RobotLifecycle::Stopping;
    else if (IsRobotStarting())
        status.runtime.lifecycle = protocol::v3::RobotLifecycle::Starting;
    else if (IsRobotRunning())
        status.runtime.lifecycle = protocol::v3::RobotLifecycle::Running;
    else
        status.runtime.lifecycle = protocol::v3::RobotLifecycle::Stopped;
    status.runtime.mode =
        IsAutoMode() ? protocol::v3::RobotMode::Automatic : protocol::v3::RobotMode::Manual;
    status.runtime.active_source = ActiveSource();
    status.runtime.flags =
        (snapshot.connection_state != device::beckhoff::SnapshotConnectionState::Disconnected
             ? 1u << 0
             : 0u) |
        (IsLogging() ? 1u << 1 : 0u) | (m_command_fresh.load() ? 1u << 2 : 0u);
    status.runtime.lifecycle_changed_unix_ns = LifecycleChangedUnixNs();
    status.runtime.accepted_command_received_unix_ns = AcceptedCommandReceivedUnixNs();

    status.beckhoff_common.move_state =
        static_cast<protocol::v3::BeckhoffMoveState>(snapshot.move_state);
    status.beckhoff_common.output_switches = snapshot.output_switches;
    status.beckhoff_common.power_level = snapshot.power_level;
    status.beckhoff_common.prepare_state = snapshot.prepare_state;
    status.beckhoff_common.error_flags = snapshot.error_flags;
    status.beckhoff_common.drive_errors = snapshot.drive_errors;
    status.beckhoff_common.motor_errors = snapshot.motor_errors;
    status.beckhoff_common.scope_type = snapshot.scope_type;
    status.beckhoff_common.values = snapshot.common_values;
    status.ercp_state.flags = snapshot.ercp_flags;
    status.ercp_state.drive_errors = snapshot.ercp_drive_errors;
    status.ercp_state.motor_errors = snapshot.ercp_motor_errors;
    status.ercp_state.type = static_cast<protocol::v3::ErcpDeviceType>(snapshot.ercp_type);
    status.ercp_state.move_status =
        static_cast<protocol::v3::ErcpMoveState>(snapshot.ercp_move_status);
    status.ercp_feedback.ercp_deliver_force = snapshot.ercp_deliver_force;
    status.ercp_feedback.guide_wire_force = snapshot.guide_wire_force;
    status.ercp_feedback.bow_force = snapshot.bow_force;
    status.ercp_feedback.ercp_deliver_position = snapshot.ercp_deliver_position;
    status.ercp_feedback.guide_wire_position = snapshot.guide_wire_position;
    status.ercp_feedback.inject_current_position_01 = snapshot.inject_current_position_01;
    status.ercp_feedback.inject_current_position_02 = snapshot.inject_current_position_02;
    status.ercp_feedback.inject_state_01 =
        static_cast<protocol::v3::InjectorState>(snapshot.inject_state_01);
    status.ercp_feedback.inject_state_02 =
        static_cast<protocol::v3::InjectorState>(snapshot.inject_state_02);
    status.ercp_feedback.balloon_pressure = snapshot.balloon_pressure;
    status.ercp_feedback.operator_position = snapshot.operator_position;
    const auto applied_commands = AppliedCommands();
    status.applied_command = applied_commands;

    status.ads_diagnostics.snapshot_sequence = snapshot.sequence;
    status.ads_diagnostics.poll_started_unix_ns = snapshot.poll_started_unix_ns;
    status.ads_diagnostics.poll_completed_unix_ns = snapshot.poll_completed_unix_ns;
    status.ads_diagnostics.snapshot_published_unix_ns = snapshot.published_unix_ns;
    status.ads_diagnostics.connection_state =
        static_cast<protocol::v3::AdsConnectionState>(snapshot.connection_state);
    status.ads_diagnostics.valid_groups = snapshot.valid_groups;
    status.ads_diagnostics.stale_groups = snapshot.stale_groups;
    status.ads_diagnostics.consecutive_failed_polls = snapshot.consecutive_failed_polls;
    status.ads_diagnostics.overall_ads_error = snapshot.overall_ads_error;
    status.ads_diagnostics.common_ads_error = snapshot.common_ads_error;
    status.ads_diagnostics.reserved_ads_error = 0;
    status.ads_diagnostics.ercp_state_ads_error = snapshot.ercp_state_ads_error;
    status.ads_diagnostics.ercp_feedback_ads_error = snapshot.ercp_feedback_ads_error;
    status.ads_diagnostics.command_write_ads_error =
        applied_commands.latest_write_attempt.ads_error;

    // 阶段三：清理非有限数值，并把对应状态组标记为过期，避免 NaN 进入 wire 数据。
    for (double &value : status.beckhoff_common.values) {
        if (!std::isfinite(value)) {
            value = 0;
            status.ads_diagnostics.stale_groups |= device::beckhoff::SnapshotCommon;
        }
    }
    double *ercpValues[] = {&status.ercp_feedback.ercp_deliver_force,
                            &status.ercp_feedback.guide_wire_force,
                            &status.ercp_feedback.bow_force,
                            &status.ercp_feedback.ercp_deliver_position,
                            &status.ercp_feedback.guide_wire_position,
                            &status.ercp_feedback.inject_current_position_01,
                            &status.ercp_feedback.inject_current_position_02,
                            &status.ercp_feedback.operator_position};
    for (double *value : ercpValues) {
        if (!std::isfinite(*value)) {
            *value = 0;
            status.ads_diagnostics.stale_groups |= device::beckhoff::SnapshotErcpFeedback;
        }
    }

    // 阶段四：填充采样时间和 V3 头，编码成功后才推进“已发送采样”标记。
    status.sampled_at_unix_ns = {now,
                                 snapshot.sampled_at_unix_ns[0],
                                 snapshot.sampled_at_unix_ns[1],
                                 snapshot.sampled_at_unix_ns[2],
                                 snapshot.sampled_at_unix_ns[3],
                                 now,
                                 snapshot.published_unix_ns,
                                 now};

    protocol::v3::Header header;
    header.message_type = protocol::v3::MessageType::RobotStatus;
    header.source = protocol::v3::Source::Robot;
    header.session_id = m_status_session_id;
    header.sequence = m_status_sequence++;
    header.sent_at_unix_ns = now;

    protocol::v3::Bytes packet;
    std::string error;
    if (!protocol::v3::encodeFullStatus(header, status, packet, &error)) {
        ROBOT_ERROR(GetSettings().Basic.Verbose() > 0,
                    fmt::format("Robot V3 status encode failed: {}", error))
        packet.clear();
    } else {
        m_last_sent_common_sample_unix_ns = common_sample_unix_ns;
    }
    return packet;
}

} // namespace ercp
