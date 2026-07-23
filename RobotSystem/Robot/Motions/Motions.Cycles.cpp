#include "robot_config.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "yunsbot_config.h"
#include "../YunSBot.h"
namespace ercp {
    void YunSBot::_base::ControlRunnable2(double t) {
        auto& robot = GetRobot();
        protocol::v2::ControlPayload command = robot_udp_v2::ZeroControl();
        robot_udp_v2::CommandMetadata metadata;
        bool fresh = false;
        const bool automaticMode = m_RobotAutoMode.load();
        const auto selectedSource = robot_udp_v2::SelectedControlSource(automaticMode);
        metadata.source = selectedSource;
        m_active_source = static_cast<std::uint16_t>(selectedSource);

        if (automaticMode) {
            fresh = parent.situaware.GetCommand(command, metadata, 0.1);
        } else {
            fresh = parent.master.GetCommand(command, metadata, 0.1);
        }

        if (fresh) {
            m_accepted_command_received_unix_ns = metadata.received_unix_ns;
        } else {
            auto &channel = automaticMode ? parent.situaware : parent.master;
            channel.LatestCommand(command, metadata);
            command = robot_udp_v2::ZeroControl();
        }
        m_command_fresh = fresh;

        //写入是否操作ERCP
        bool ercp_online = robot.BeckhoffIsERCPOnline();
        bool ercp_ready = robot.BeckhoffIsERCPReady();
        static bool bERCPOnline = ercp_online;
        static bool bERCPReady = false;
        if (ercp_online != bERCPOnline) {
            if (!ercp_online)
                robot.BeckhoffERCPOperateState(false);
            else if (ercp_ready)
                robot.BeckhoffERCPOperateState(true);
        } else if (ercp_online && ercp_ready != bERCPReady) {
            robot.BeckhoffERCPOperateState(ercp_ready);
        }
        bERCPOnline = ercp_online;
        bERCPReady = ercp_ready;

        // 只在此处将网络控制字映射为 PLC native 10+6。
        beckhoff_follow_cmd follow_cmd;
        build_follow_cmd(command, follow_cmd);
        const auto appliedAt = robot_udp_v2::UnixNowNs();
        const auto adsError = robot.BeckhoffFollowDataResult(sizeof(follow_cmd), &follow_cmd);
        const bool succeeded = adsError == 0;
        // A timeout describes a successfully written safety-zero command. If the
        // ADS write itself fails, the wire status must report that failure first.
        const auto result = !succeeded
            ? protocol::v2::ApplyResult::Failed
            : (fresh ? protocol::v2::ApplyResult::Succeeded
                     : protocol::v2::ApplyResult::TimedOutToZero);
        m_applied_commands.MarkAttempt(
            command, metadata, result, adsError, appliedAt, succeeded);
    }

    // 建立跟随命令信息
    void YunSBot::_base::build_follow_cmd(
        const protocol::v2::ControlPayload &cmd, beckhoff_follow_cmd &follow_cmd)
    {
        follow_cmd.follow_comp_botton = cmd.values[0];
        follow_cmd.vel_move = cmd.values[1];
        follow_cmd.vel_rotate = cmd.values[2];
        follow_cmd.vel_bend_lr = cmd.values[3];
        follow_cmd.vel_bend_ud = cmd.values[4];
        follow_cmd.vel_pincer = cmd.values[5];
        follow_cmd.vel_cutter_feed = cmd.values[6];
        follow_cmd.vel_cutter_rot = cmd.values[7];
        follow_cmd.vel_cutter_bend = cmd.values[8];
        follow_cmd.vel_wire_feed = cmd.values[9];
        follow_cmd.home_rotate = (cmd.switches & (1u << 0)) != 0;
        follow_cmd.home_bend_lr = (cmd.switches & (1u << 1)) != 0;
        follow_cmd.home_bend_ud = (cmd.switches & (1u << 2)) != 0;
        follow_cmd.switch_water = (cmd.switches & (1u << 3)) != 0;
        follow_cmd.switch_gas = (cmd.switches & (1u << 4)) != 0;
        follow_cmd.switch_suct = (cmd.switches & (1u << 5)) != 0;
    }


} // namespace ercp

