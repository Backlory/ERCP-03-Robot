#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "beckhoff_snapshot.hpp"
#include "protocol/robot_udp_v3.hpp"
#include "yunsbot_config.h"

namespace ercp::control_cycle {

struct SourceDecision {
    protocol::v3::Source source = protocol::v3::Source::Master;
    bool master_override = false;
};

/**
 * @brief 根据自动模式和 Master 优先窗口选择本周期的控制来源。
 * @details 自动模式下只有满足优先开关和时间窗口时才保留 Master，否则使用 Cloud；手动模式始终使用 Master。
 */
inline SourceDecision ChooseSource(bool automatic_mode,
                                   bool master_priority_enabled,
                                   bool master_command_within_priority_window)
{
    const bool master_override =
        automatic_mode && master_priority_enabled && master_command_within_priority_window;
    return {automatic_mode && !master_override ? protocol::v3::Source::Cloud
                                               : protocol::v3::Source::Master,
            master_override};
}

struct PreparedCommands {
    protocol::v3::ControlPayload safe_control;
    device::beckhoff::GoldDiscreteCommand discrete;
    bool ercp_allowed = false;
};

/**
 * @brief 对网络控制命令执行新鲜度、ERCP 状态和注射器状态安全门控。
 * @details 先生成可安全下发的连续控制量，再映射 ERCP 离散字段；过期或未就绪时将危险动作归零。
 */
inline PreparedCommands PrepareCommands(const protocol::v3::ControlPayload &control,
                                         bool fresh,
                                         bool ercp_online,
                                         bool ercp_ready,
                                         const device::beckhoff::BeckhoffSnapshot &snapshot)
{
    using protocol::v3::ControlValueIndex;
    using protocol::v3::controlIndex;

    // 阶段一：复制原始控制量，并根据命令新鲜度和 ERCP 状态决定危险动作是否允许。
    PreparedCommands prepared;
    prepared.safe_control = control;
    prepared.ercp_allowed = fresh && ercp_online && ercp_ready;
    if (!prepared.ercp_allowed) {
        prepared.safe_control.values[controlIndex(ControlValueIndex::CutterFeed)] = 0;
        prepared.safe_control.values[controlIndex(ControlValueIndex::CutterSwing)] = 0;
        prepared.safe_control.values[controlIndex(ControlValueIndex::CutterBend)] = 0;
        prepared.safe_control.values[controlIndex(ControlValueIndex::GuideWireFeed)] = 0;
    }

    // 阶段二：把安全后的协议字段映射为 Beckhoff 适配器使用的离散命令布局。
    auto &discrete = prepared.discrete;
    discrete.robot_action = control.robot_action;
    discrete.operate = prepared.ercp_allowed && (control.ercp_switches & (1u << 0)) != 0;
    discrete.cooperate = prepared.ercp_allowed && (control.ercp_switches & (1u << 1)) != 0;
    for (std::size_t i = 0; i < 6; ++i)
        discrete.handle_6d[i] = prepared.ercp_allowed ? control.ercp_6d[i] : 0.0;
    for (std::size_t i = 0; i < 3; ++i)
        discrete.buttons[i] =
            prepared.ercp_allowed && (control.ercp_switches & (1u << (i + 2))) != 0;
    for (std::size_t i = 0; i < 2; ++i) {
        discrete.inject_velocity[i] = prepared.ercp_allowed ? control.inject_velocity[i] : 0.0;
        discrete.inject_position[i] = prepared.ercp_allowed ? control.inject_position[i] : 0.0;
        discrete.inject_enable[i] =
            prepared.ercp_allowed && (control.inject_enables & (1u << i)) != 0;
    }

    // 阶段三：已完成的注射器在状态变化前禁止再次使能，避免重复触发 PLC 动作。
    if (snapshot.inject_state_01 == 11)
        discrete.inject_enable[0] = false;
    if (snapshot.inject_state_02 == 11)
        discrete.inject_enable[1] = false;
    return prepared;
}

/**
 * @brief 将协议连续控制量和开关位转换为 Beckhoff follow 命令结构。
 * @details 连续量先限制在协议允许范围，再按固定字段顺序写入；开关位最后映射为 BOOL 成员。
 */
inline beckhoff_follow_cmd ToFollowCommand(const protocol::v3::ControlPayload &control)
{
    using protocol::v3::ControlValueIndex;
    using protocol::v3::controlIndex;

    // 阶段一：统一限制连续控制量，避免网络输入越过设备命令允许范围。
    constexpr double kControlValueLo = -1.0;
    constexpr double kControlValueHi = +1.0;
    const auto value = [&](ControlValueIndex index) {
        return std::clamp(control.values[controlIndex(index)], kControlValueLo, kControlValueHi);
    };

    // 阶段二：清零设备结构体并按协议索引映射运动量。
    beckhoff_follow_cmd command;
    std::memset(&command, 0, sizeof(command));
    command.follow_comp_botton = value(ControlValueIndex::FollowCompensation);
    command.vel_move = value(ControlValueIndex::ScopeMove);
    command.vel_rotate = value(ControlValueIndex::ScopeRotate);
    command.vel_bend_lr = value(ControlValueIndex::ScopeBendLr);
    command.vel_bend_ud = value(ControlValueIndex::ScopeBendUd);
    command.vel_pincer = value(ControlValueIndex::Pincer);
    command.vel_cutter_feed = value(ControlValueIndex::CutterFeed);
    command.vel_cutter_rot = value(ControlValueIndex::CutterSwing);
    command.vel_cutter_bend = value(ControlValueIndex::CutterBend);
    command.vel_wire_feed = value(ControlValueIndex::GuideWireFeed);
    // 阶段三：把协议开关位映射到回零和水/气/吸取控制字段。
    command.home_rotate = (control.switches & (1u << 0)) != 0;
    command.home_bend_lr = (control.switches & (1u << 1)) != 0;
    command.home_bend_ud = (control.switches & (1u << 2)) != 0;
    command.switch_water = (control.switches & (1u << 3)) != 0;
    command.switch_gas = (control.switches & (1u << 4)) != 0;
    command.switch_suct = (control.switches & (1u << 5)) != 0;
    return command;
}

/**
 * @brief 根据实际下发的 follow 和离散命令重建应用命令审计载荷。
 * @details 该结果反映安全门控、限幅和变化检测后的设备侧命令，不再直接等同于网络原始输入。
 */
inline protocol::v3::ControlPayload
ToAppliedControl(const beckhoff_follow_cmd &follow,
                 const device::beckhoff::GoldDiscreteCommand &discrete)
{
    using protocol::v3::ControlValueIndex;
    using protocol::v3::controlIndex;

    // 阶段一：把 follow 结构中的连续字段和基本开关还原为协议语义。
    protocol::v3::ControlPayload applied;
    applied.robot_action = discrete.robot_action;
    applied.values[controlIndex(ControlValueIndex::FollowCompensation)] = follow.follow_comp_botton;
    applied.values[controlIndex(ControlValueIndex::ScopeMove)] = follow.vel_move;
    applied.values[controlIndex(ControlValueIndex::ScopeRotate)] = follow.vel_rotate;
    applied.values[controlIndex(ControlValueIndex::ScopeBendLr)] = follow.vel_bend_lr;
    applied.values[controlIndex(ControlValueIndex::ScopeBendUd)] = follow.vel_bend_ud;
    applied.values[controlIndex(ControlValueIndex::Pincer)] = follow.vel_pincer;
    applied.values[controlIndex(ControlValueIndex::CutterFeed)] = follow.vel_cutter_feed;
    applied.values[controlIndex(ControlValueIndex::CutterSwing)] = follow.vel_cutter_rot;
    applied.values[controlIndex(ControlValueIndex::CutterBend)] = follow.vel_cutter_bend;
    applied.values[controlIndex(ControlValueIndex::GuideWireFeed)] = follow.vel_wire_feed;
    applied.switches =
        (follow.home_rotate ? 1u << 0 : 0u) | (follow.home_bend_lr ? 1u << 1 : 0u) |
        (follow.home_bend_ud ? 1u << 2 : 0u) | (follow.switch_water ? 1u << 3 : 0u) |
        (follow.switch_gas ? 1u << 4 : 0u) | (follow.switch_suct ? 1u << 5 : 0u);
    // 阶段二：把 ERCP 离散命令和数组复制回审计载荷。
    applied.ercp_switches =
        (discrete.operate ? 1u << 0 : 0u) | (discrete.cooperate ? 1u << 1 : 0u) |
        (discrete.buttons[0] ? 1u << 2 : 0u) | (discrete.buttons[1] ? 1u << 3 : 0u) |
        (discrete.buttons[2] ? 1u << 4 : 0u);
    std::copy(std::begin(discrete.handle_6d), std::end(discrete.handle_6d), applied.ercp_6d.begin());
    std::copy(std::begin(discrete.inject_velocity),
              std::end(discrete.inject_velocity),
              applied.inject_velocity.begin());
    std::copy(std::begin(discrete.inject_position),
              std::end(discrete.inject_position),
              applied.inject_position.begin());
    applied.inject_enables =
        (discrete.inject_enable[0] ? 1u << 0 : 0u) |
        (discrete.inject_enable[1] ? 1u << 1 : 0u);
    return applied;
}

inline std::uint32_t FirstAdsError(std::uint32_t first, std::uint32_t second)
{
    return first == 0 ? second : first;
}

inline protocol::v3::ApplyResult ClassifyApplyResult(bool fresh, std::uint32_t ads_error)
{
    if (ads_error != 0)
        return protocol::v3::ApplyResult::Failed;
    return fresh ? protocol::v3::ApplyResult::Succeeded
                 : protocol::v3::ApplyResult::TimedOutToZero;
}

} // namespace ercp::control_cycle
