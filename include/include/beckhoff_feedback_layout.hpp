#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "beckhoff_snapshot.hpp"

namespace device::beckhoff {

constexpr std::size_t RobotFeedbackBlockSize = 320;

// Exact ADS block layout of MAIN.Info_Feedback_ToMaster on the deployed
// TwinCAT runtime. The PLC declaration order, not the gold-table display
// order, determines these offsets.
struct RobotFeedbackData {
    double Follow_Length;
    bool Switch_Water;
    bool Switch_Gas;
    bool Switch_Suck;
    double Axes_Pos[21];
    double Big_Whell;
    double Small_Whell;
    double Force_Sensor[10];
    std::int16_t Power_level;
    double lifter;
    double Deliver_force;
    double Rotate_Deqree;
    double Follow_Force;
};

static_assert(sizeof(RobotFeedbackData) == RobotFeedbackBlockSize,
    "RobotFeedbackData must match MAIN.Info_Feedback_ToMaster");
static_assert(offsetof(RobotFeedbackData, Follow_Length) == 0,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Switch_Water) == 8,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Switch_Gas) == 9,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Switch_Suck) == 10,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Axes_Pos) == 16,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Big_Whell) == 184,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Small_Whell) == 192,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Force_Sensor) == 200,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Power_level) == 280,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, lifter) == 288,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Deliver_force) == 296,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Rotate_Deqree) == 304,
    "RobotFeedbackData ABI drift");
static_assert(offsetof(RobotFeedbackData, Follow_Force) == 312,
    "RobotFeedbackData ABI drift");

inline bool DecodeRobotFeedbackBlock(
    const void *block, std::size_t blockSize, RobotFeedbackData &feedback)
{
    if (block == nullptr || blockSize != RobotFeedbackBlockSize) return false;

    const auto *bytes = static_cast<const std::uint8_t *>(block);
    std::memcpy(&feedback.Follow_Length, bytes + 0, sizeof(feedback.Follow_Length));
    std::memcpy(&feedback.Switch_Water, bytes + 8, sizeof(feedback.Switch_Water));
    std::memcpy(&feedback.Switch_Gas, bytes + 9, sizeof(feedback.Switch_Gas));
    std::memcpy(&feedback.Switch_Suck, bytes + 10, sizeof(feedback.Switch_Suck));
    std::memcpy(&feedback.Axes_Pos, bytes + 16, sizeof(feedback.Axes_Pos));
    std::memcpy(&feedback.Big_Whell, bytes + 184, sizeof(feedback.Big_Whell));
    std::memcpy(&feedback.Small_Whell, bytes + 192, sizeof(feedback.Small_Whell));
    std::memcpy(&feedback.Force_Sensor, bytes + 200, sizeof(feedback.Force_Sensor));
    std::memcpy(&feedback.Power_level, bytes + 280, sizeof(feedback.Power_level));
    std::memcpy(&feedback.lifter, bytes + 288, sizeof(feedback.lifter));
    std::memcpy(&feedback.Deliver_force, bytes + 296, sizeof(feedback.Deliver_force));
    std::memcpy(&feedback.Rotate_Deqree, bytes + 304, sizeof(feedback.Rotate_Deqree));
    std::memcpy(&feedback.Follow_Force, bytes + 312, sizeof(feedback.Follow_Force));
    return true;
}

inline void ApplyRobotFeedback(
    const RobotFeedbackData &feedback, BeckhoffSnapshot &snapshot)
{
    snapshot.output_switches = static_cast<std::uint16_t>(
        (feedback.Switch_Water ? 1u << 0 : 0u)
        | (feedback.Switch_Gas ? 1u << 1 : 0u)
        | (feedback.Switch_Suck ? 1u << 2 : 0u));
    snapshot.power_level = feedback.Power_level;
    snapshot.common_values[0] = feedback.Follow_Length;
    snapshot.common_values[1] = feedback.Big_Whell;
    snapshot.common_values[2] = feedback.Small_Whell;
    std::copy(std::begin(feedback.Force_Sensor), std::end(feedback.Force_Sensor),
        snapshot.common_values.begin() + 3);
    snapshot.common_values[13] = feedback.lifter;
    snapshot.common_values[14] = feedback.Deliver_force;
    snapshot.common_values[15] = feedback.Rotate_Deqree;
    snapshot.common_values[16] = feedback.Follow_Force;
    // V3 preserves its fixed 19-axis wire contract while TwinCAT exposes 21.
    std::copy_n(std::begin(feedback.Axes_Pos), 19,
        snapshot.common_values.begin() + 17);
}

} // namespace device::beckhoff
