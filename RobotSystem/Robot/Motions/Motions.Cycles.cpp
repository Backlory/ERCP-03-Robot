#include <algorithm>
#include <cstring>

#include "robot_config.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "yunsbot_config.h"
#include "../YunSBot.h"
namespace ercp {

    // M2: 控制量量程 clamp,量程表见 docs/robot-udp-v2.md §4.1。
    // 10 路连续控制量统一为归一化速度比 [-1.0, +1.0]:依据当前唯一生产发送端
    // (Master 手柄映射,win_joystick 轴归一化输出 ±1、按键映射 ±1)。
    // 注意:此为占位量程,PLC 工程单位与每路物理量程待 TwinCAT 权威 schema
    // 与设备安全策略确认后收紧。codec 已拒绝 NaN/Inf,此处输入保证 finite。
    static constexpr double kControlValueLo = -1.0;
    static constexpr double kControlValueHi = +1.0;
    static inline double clamp_control(double v)
    {
        return std::clamp(v, kControlValueLo, kControlValueHi);
    }

    static protocol::v2::ControlPayload applied_control(
        const beckhoff_follow_cmd &follow_cmd, const protocol::v2::ControlPayload &requested)
    {
        using protocol::v2::ControlValueIndex;
        using protocol::v2::controlIndex;
        protocol::v2::ControlPayload applied = requested;
        applied.values[controlIndex(ControlValueIndex::FollowCompensation)] =
            follow_cmd.follow_comp_botton;
        applied.values[controlIndex(ControlValueIndex::ScopeMove)] = follow_cmd.vel_move;
        applied.values[controlIndex(ControlValueIndex::ScopeRotate)] = follow_cmd.vel_rotate;
        applied.values[controlIndex(ControlValueIndex::ScopeBendLr)] = follow_cmd.vel_bend_lr;
        applied.values[controlIndex(ControlValueIndex::ScopeBendUd)] = follow_cmd.vel_bend_ud;
        applied.values[controlIndex(ControlValueIndex::Pincer)] = follow_cmd.vel_pincer;
        applied.values[controlIndex(ControlValueIndex::CutterFeed)] =
            follow_cmd.vel_cutter_feed;
        applied.values[controlIndex(ControlValueIndex::CutterSwing)] =
            follow_cmd.vel_cutter_rot;
        applied.values[controlIndex(ControlValueIndex::CutterBend)] =
            follow_cmd.vel_cutter_bend;
        applied.values[controlIndex(ControlValueIndex::GuideWireFeed)] =
            follow_cmd.vel_wire_feed;
        applied.switches =
            (follow_cmd.home_rotate ? 1u << 0 : 0u)
            | (follow_cmd.home_bend_lr ? 1u << 1 : 0u)
            | (follow_cmd.home_bend_ud ? 1u << 2 : 0u)
            | (follow_cmd.switch_water ? 1u << 3 : 0u)
            | (follow_cmd.switch_gas ? 1u << 4 : 0u)
            | (follow_cmd.switch_suct ? 1u << 5 : 0u);
        return applied;
    }

    void YunSBot::_base::ControlRunnable2(double t) {
        auto& robot = GetRobot();
        protocol::v2::ControlPayload command = robot_udp_v2::ZeroControl();
        robot_udp_v2::CommandMetadata metadata;
        bool fresh = false;
        const bool automaticMode = m_RobotAutoMode.load();

        // Task 11: Master 强制优先仲裁。自主(auto)模式下,若最近一帧 Master(31002) 命令
        // 距今 < 200ms,视为人类介入,本周期丢弃 Cloud 改用 Master(仍按 0.1s fresh 逻辑回退)。
        // 200ms 判定复用 receiver 内 mutex 保护的 latest_received_(仅在命令通过完整校验后更新),
        // 经 GetCommand 线程安全读取,无需另存时间点。开关(m_master_priority)关闭时回退原硬二选一。
        constexpr auto kMasterPriorityWindow = std::chrono::milliseconds(200);
        constexpr double kMasterPriorityWindowSeconds =
            std::chrono::duration<double>(kMasterPriorityWindow).count();
        bool masterOverride = false;
        if (automaticMode && m_master_priority.load()) {
            protocol::v2::ControlPayload probeCommand;
            robot_udp_v2::CommandMetadata probeMetadata;
            if (parent.master.GetCommand(
                    probeCommand, probeMetadata, kMasterPriorityWindowSeconds)) {
                masterOverride = true;
                m_master_priority_overrides.fetch_add(1);
                // Cloud 命令因仲裁被丢弃,节流日志便于联调(复用 stats 日志风格)。
                static auto lastOverrideLog = std::chrono::steady_clock::now();
                const auto nowLog = std::chrono::steady_clock::now();
                if (GetSettings().Basic.Verbose() > 0
                    && nowLog - lastOverrideLog >= std::chrono::seconds(10)) {
                    ROBOT_INFO(true, fmt::format(
                        "Robot V2 master-priority override: cloud command dropped, overrides={}",
                        m_master_priority_overrides.load()))
                    lastOverrideLog = nowLog;
                }
            }
        }

        const bool useMaster = !automaticMode || masterOverride;
        const auto selectedSource = useMaster
            ? protocol::v2::Source::Master : protocol::v2::Source::Cloud;
        metadata.source = selectedSource;

        if (useMaster) {
            fresh = parent.master.GetCommand(command, metadata, 0.1);
        } else {
            fresh = parent.situaware.GetCommand(command, metadata, 0.1);
        }
        // L16: 无新鲜命令(与 flags bit2 同一 fresh 判定)时 active_source 上报 None=0,
        // 不再声称某个来源"正在控制"。
        m_active_source = fresh ? static_cast<std::uint16_t>(selectedSource) : 0u;

        if (fresh) {
            m_accepted_command_received_unix_ns = metadata.received_unix_ns;
        } else {
            auto &channel = useMaster ? parent.master : parent.situaware;
            channel.LatestCommand(command, metadata);
            command = robot_udp_v2::ZeroControl();
        }
        m_command_fresh = fresh;

        // 只在此处将网络控制字映射为 PLC native 10+6。
        beckhoff_follow_cmd follow_cmd;
        build_follow_cmd(command, follow_cmd);
        const auto appliedAt = robot_udp_v2::UnixNowNs();
        auto adsError = robot.BeckhoffFollowDataResult(sizeof(follow_cmd), &follow_cmd);
        device::beckhoff::GoldDiscreteCommand discrete;
        discrete.robot_action = command.robot_action;
        const bool ercpAllowed = fresh
            && robot.BeckhoffIsERCPOnline() && robot.BeckhoffIsERCPReady();
        discrete.operate = ercpAllowed && (command.ercp_switches & (1u << 0)) != 0;
        discrete.cooperate = ercpAllowed && (command.ercp_switches & (1u << 1)) != 0;
        for (std::size_t i = 0; i < 6; ++i)
            discrete.handle_6d[i] = ercpAllowed ? command.ercp_6d[i] : 0.0;
        discrete.buttons[0] = ercpAllowed && (command.ercp_switches & (1u << 2)) != 0;
        discrete.buttons[1] = ercpAllowed && (command.ercp_switches & (1u << 3)) != 0;
        discrete.buttons[2] = ercpAllowed && (command.ercp_switches & (1u << 4)) != 0;
        for (std::size_t i = 0; i < 2; ++i) {
            discrete.inject_velocity[i] = ercpAllowed ? command.inject_velocity[i] : 0.0;
            discrete.inject_position[i] = ercpAllowed ? command.inject_position[i] : 0.0;
            discrete.inject_enable[i] =
                ercpAllowed && (command.inject_enables & (1u << i)) != 0;
        }
        const auto discreteError = robot.BeckhoffGoldDiscreteCommandResult(discrete);
        if (adsError == 0) adsError = discreteError;
        const bool succeeded = adsError == 0;
        // A timeout describes a successfully written safety-zero command. If the
        // ADS write itself fails, the wire status must report that failure first.
        const auto result = !succeeded
            ? protocol::v2::ApplyResult::Failed
            : (fresh ? protocol::v2::ApplyResult::Succeeded
                     : protocol::v2::ApplyResult::TimedOutToZero);
        // 状态中的历史命令必须等于实际交给 ADS 的 10+6，而不是限幅前的网络原值。
        const auto appliedCommand = applied_control(follow_cmd, command);
        m_applied_commands.MarkAttempt(
            appliedCommand, metadata, result, adsError, appliedAt, succeeded);
    }

    // 建立跟随命令信息
    void YunSBot::_base::build_follow_cmd(
        const protocol::v2::ControlPayload &cmd, beckhoff_follow_cmd &follow_cmd)
    {
        // L23: 88B 结构体含 2B 尾部对齐 padding,写 PLC 前整体清零,
        // 避免未初始化字节随 ADS 写入 PLC。
        std::memset(&follow_cmd, 0, sizeof(follow_cmd));
        // M2: 写 PLC 前逐路 clamp 到协议量程(docs/robot-udp-v2.md §4.1)
        using protocol::v2::ControlValueIndex;
        using protocol::v2::controlIndex;
        follow_cmd.follow_comp_botton =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::FollowCompensation)]);
        follow_cmd.vel_move = clamp_control(cmd.values[controlIndex(ControlValueIndex::ScopeMove)]);
        follow_cmd.vel_rotate =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::ScopeRotate)]);
        follow_cmd.vel_bend_lr =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::ScopeBendLr)]);
        follow_cmd.vel_bend_ud =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::ScopeBendUd)]);
        follow_cmd.vel_pincer = clamp_control(cmd.values[controlIndex(ControlValueIndex::Pincer)]);
        follow_cmd.vel_cutter_feed =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::CutterFeed)]);
        // 生产 yunsbot_config.h 将这一自由度定义为“切开刀摆转”。
        follow_cmd.vel_cutter_rot =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::CutterSwing)]);
        follow_cmd.vel_cutter_bend =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::CutterBend)]);
        follow_cmd.vel_wire_feed =
            clamp_control(cmd.values[controlIndex(ControlValueIndex::GuideWireFeed)]);
        follow_cmd.home_rotate = (cmd.switches & (1u << 0)) != 0;
        follow_cmd.home_bend_lr = (cmd.switches & (1u << 1)) != 0;
        follow_cmd.home_bend_ud = (cmd.switches & (1u << 2)) != 0;
        follow_cmd.switch_water = (cmd.switches & (1u << 3)) != 0;
        follow_cmd.switch_gas = (cmd.switches & (1u << 4)) != 0;
        follow_cmd.switch_suct = (cmd.switches & (1u << 5)) != 0;
    }


} // namespace ercp

