#include "robot_config.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "../control_cycle_policy.hpp"
#include "../YunSBot.h"

namespace ercp {
namespace {

/**
 * @brief 按限频策略记录 Master 优先级覆盖 Cloud 命令的诊断信息。
 * @details 控制周期可能连续触发覆盖；日志每 10 秒最多输出一次，避免周期线程被日志 I/O 拖慢。
 */
void LogMasterOverride(std::uint64_t override_count)
{
    static auto last_log = std::chrono::steady_clock::now();
    const auto now = std::chrono::steady_clock::now();
    if (GetSettings().Basic.Verbose() <= 0 || now - last_log < std::chrono::seconds(10))
        return;

    ROBOT_INFO(true,
               fmt::format("Robot V3 master-priority override: cloud command dropped, "
                           "overrides={}",
                           override_count))
    last_log = now;
}

} // namespace

/**
 * @brief 功能：执行一个机器人控制周期，选择控制源、应用安全门控并写入 Beckhoff。
 * @details 机制：按“源仲裁—新鲜度/安全命令—PLC 映射—follow 与离散写入—应用结果审计”的固定顺序处理，失败结果进入状态快照。
 */
void YunSBot::_base::ControlRunnable2(double t)
{
    (void)t;
    auto &robot = GetRobot();
    const bool automatic_mode = m_RobotAutoMode.load();

    // 阶段一：先独立读取 Master 急停，再按普通运动命令策略仲裁控制源。
    // Emergency stop is deliberately not part of Cloud/Master motion-source
    // arbitration: a Cloud command must never mask a Master stop request.
    constexpr double kMasterPriorityWindowSeconds = 0.2;
    protocol::v3::ControlPayload master_probe_command = robot_udp_v3::ZeroControl();
    robot_udp_v3::CommandMetadata master_probe_metadata;
    const bool master_command_fresh = parent.master.GetCommand(
        master_probe_command, master_probe_metadata, kMasterPriorityWindowSeconds);
    if (master_command_fresh) {
        // A fresh explicit zero is the only UDP release authority. If the
        // Master packet expires, this stored request is intentionally retained.
        SetMasterUdpEmergencyStop(master_probe_command.emergency_stop != 0);
    }
    const bool master_within_priority_window =
        automatic_mode && m_master_priority.load() && master_command_fresh;
    const auto source_decision = control_cycle::ChooseSource(automatic_mode,
                                                              m_master_priority.load(),
                                                              master_within_priority_window);
    if (source_decision.master_override) {
        const auto overrides = m_master_priority_overrides.fetch_add(1) + 1;
        LogMasterOverride(overrides);
    }

    // 阶段二：读取选中的最新命令，并保留审计所需的来源、序号和时间戳。
    const bool use_master = source_decision.source == protocol::v3::Source::Master;
    auto &channel = use_master ? parent.master : parent.situaware;
    protocol::v3::ControlPayload command = robot_udp_v3::ZeroControl();
    robot_udp_v3::CommandMetadata metadata;
    metadata.source = source_decision.source;
    const bool fresh = channel.GetCommand(command, metadata, 0.1);

    m_active_source = fresh ? static_cast<std::uint16_t>(source_decision.source) : 0u;
    if (fresh) {
        m_accepted_command_received_unix_ns = metadata.received_unix_ns;
    } else {
        channel.LatestCommand(command, metadata);
        command = robot_udp_v3::ZeroControl();
    }
    m_command_fresh = fresh;

    // Force the ordinary motion command to zero while either emergency-stop
    // authority is active. The audit field is kept asserted so status reflects
    // the safety decision even when Cloud is the selected motion source.
    if (EmergencyStopActive()) {
        command = robot_udp_v3::ZeroControl();
        command.emergency_stop = 1;
    }

    // 阶段三：应用安全策略，把领域命令映射为两种 PLC 写入布局。
    const auto prepared = control_cycle::PrepareCommands(command,
                                                          fresh,
                                                          robot.BeckhoffIsERCPOnline(),
                                                          robot.BeckhoffIsERCPReady(),
                                                          robot.BeckhoffSnapshot());
    const auto follow_command = control_cycle::ToFollowCommand(prepared.safe_control);

    // 阶段四：执行一次 follow 写入和一次离散写入，并保留首个 ADS 错误。
    const auto applied_at = robot_udp_v3::UnixNowNs();
    const auto follow_error = robot.BeckhoffWriteFollowCommand(follow_command);
    const auto discrete_error = robot.BeckhoffGoldDiscreteCommandResult(prepared.discrete);
    const auto ads_error = control_cycle::FirstAdsError(follow_error, discrete_error);

    // 阶段五：记录经过安全门控和限幅后真正送到 PLC 适配器的命令。
    auto applied_command =
        control_cycle::ToAppliedControl(follow_command, prepared.discrete);
    applied_command.emergency_stop = command.emergency_stop;
    // The applied-command record has the same authority rule as a wire
    // control packet. If Cloud is the selected motion source, its audit record
    // must not claim a Master-only emergency field, even when the independent
    // safety latch forced the physical command to zero.
    if (metadata.source != protocol::v3::Source::Master)
        applied_command.emergency_stop = 0;
    m_applied_commands.MarkAttempt(applied_command,
                                   metadata,
                                   control_cycle::ClassifyApplyResult(fresh, ads_error),
                                   ads_error,
                                   applied_at,
                                   ads_error == 0);
}

} // namespace ercp
