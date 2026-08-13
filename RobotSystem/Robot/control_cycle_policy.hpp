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

inline PreparedCommands PrepareCommands(const protocol::v3::ControlPayload &control,
                                         bool fresh,
                                         bool ercp_online,
                                         bool ercp_ready,
                                         const device::beckhoff::BeckhoffSnapshot &snapshot)
{
    using protocol::v3::ControlValueIndex;
    using protocol::v3::controlIndex;

    PreparedCommands prepared;
    prepared.safe_control = control;
    prepared.ercp_allowed = fresh && ercp_online && ercp_ready;
    if (!prepared.ercp_allowed) {
        prepared.safe_control.values[controlIndex(ControlValueIndex::CutterFeed)] = 0;
        prepared.safe_control.values[controlIndex(ControlValueIndex::CutterSwing)] = 0;
        prepared.safe_control.values[controlIndex(ControlValueIndex::CutterBend)] = 0;
        prepared.safe_control.values[controlIndex(ControlValueIndex::GuideWireFeed)] = 0;
    }

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

    // A completed injector cannot be enabled again until its state changes.
    if (snapshot.inject_state_01 == 11)
        discrete.inject_enable[0] = false;
    if (snapshot.inject_state_02 == 11)
        discrete.inject_enable[1] = false;
    return prepared;
}

inline beckhoff_follow_cmd ToFollowCommand(const protocol::v3::ControlPayload &control)
{
    using protocol::v3::ControlValueIndex;
    using protocol::v3::controlIndex;

    constexpr double kControlValueLo = -1.0;
    constexpr double kControlValueHi = +1.0;
    const auto value = [&](ControlValueIndex index) {
        return std::clamp(control.values[controlIndex(index)], kControlValueLo, kControlValueHi);
    };

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
    command.home_rotate = (control.switches & (1u << 0)) != 0;
    command.home_bend_lr = (control.switches & (1u << 1)) != 0;
    command.home_bend_ud = (control.switches & (1u << 2)) != 0;
    command.switch_water = (control.switches & (1u << 3)) != 0;
    command.switch_gas = (control.switches & (1u << 4)) != 0;
    command.switch_suct = (control.switches & (1u << 5)) != 0;
    return command;
}

inline protocol::v3::ControlPayload
ToAppliedControl(const beckhoff_follow_cmd &follow,
                 const device::beckhoff::GoldDiscreteCommand &discrete)
{
    using protocol::v3::ControlValueIndex;
    using protocol::v3::controlIndex;

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
