#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "beckhoff_snapshot_policy.hpp"

namespace device::beckhoff {

// These are RobotSystem's local publication capacities. They are not ADS
// array lengths and must not be used as a request size for a PLC array.
constexpr std::size_t kRobotPublishedAxisCount = 19;
constexpr std::size_t kRobotForceSensorCount = 10;

// TwinCAT scalar sizes used by the read table. Keep these independent from
// the C++ representation of an array or a PLC STRUCT.
constexpr unsigned long kAdsBoolBytes = 1;
constexpr unsigned long kAdsInt16Bytes = 2;
constexpr unsigned long kAdsInt32Bytes = 4;
constexpr unsigned long kAdsLrealBytes = 8;

// Values decoded from individual MAIN.Info_Feedback_ToMaster leaf symbols.
// This is a local value model, not a mirror of the PLC STRUCT ABI.
struct RobotFeedbackLeaves {
    double follow_length = 0;
    bool switch_water = false;
    bool switch_gas = false;
    bool switch_suck = false;
    std::array<double, kRobotPublishedAxisCount> axes_pos{};
    double big_wheel = 0;
    double small_wheel = 0;
    std::array<double, kRobotForceSensorCount> force_sensor{};
    std::int16_t power_level = 0;
    double lifter = 0;
    double deliver_force = 0;
    double rotate_degree = 0;
    double follow_force = 0;
};

// 每个请求保留独立 ADS 错误码，用于诊断完整反馈读取失败的原因。
constexpr std::size_t kFeedbackFollowLengthIndex = 0;
constexpr std::size_t kFeedbackSwitchWaterIndex = 1;
constexpr std::size_t kFeedbackSwitchGasIndex = 2;
constexpr std::size_t kFeedbackSwitchSuckIndex = 3;
constexpr std::size_t kFeedbackBigWheelIndex = 4;
constexpr std::size_t kFeedbackSmallWheelIndex = 5;
constexpr std::size_t kFeedbackForceSensorBaseIndex = 6;
constexpr std::size_t kFeedbackPowerLevelIndex =
    kFeedbackForceSensorBaseIndex + kRobotForceSensorCount;
constexpr std::size_t kFeedbackLifterIndex = kFeedbackPowerLevelIndex + 1;
constexpr std::size_t kFeedbackDeliverForceIndex = kFeedbackLifterIndex + 1;
constexpr std::size_t kFeedbackRotateDegreeIndex = kFeedbackDeliverForceIndex + 1;
constexpr std::size_t kFeedbackFollowForceIndex = kFeedbackRotateDegreeIndex + 1;
constexpr std::size_t kFeedbackAxesBaseIndex = kFeedbackFollowForceIndex + 1;
constexpr std::size_t kRobotFeedbackLeafCount =
    kFeedbackAxesBaseIndex + kRobotPublishedAxisCount;

using RobotFeedbackLeafErrors = std::array<std::uint32_t, kRobotFeedbackLeafCount>;

/**
 * @brief 将完整取得的 Beckhoff 反馈映射到统一状态快照。
 * @details 任意叶字段读取失败时清空主机器人来源；有效性及取得时间由完整轮询设置。
 */
inline void ApplyRobotFeedback(const RobotFeedbackLeaves &feedback,
                               const RobotFeedbackLeafErrors &errors,
                               BeckhoffSnapshot &snapshot)
{
    for (const auto error : errors) {
        if (error != 0) {
            ClearRobotFeedback(snapshot);
            return;
        }
    }
    snapshot.output_switches = 0;
    if (feedback.switch_water)
        snapshot.output_switches |= static_cast<std::uint16_t>(1u << 0);
    if (feedback.switch_gas)
        snapshot.output_switches |= static_cast<std::uint16_t>(1u << 1);
    if (feedback.switch_suck)
        snapshot.output_switches |= static_cast<std::uint16_t>(1u << 2);

    snapshot.power_level = feedback.power_level;
    snapshot.common_values[0] = feedback.follow_length;
    snapshot.common_values[1] = feedback.big_wheel;
    snapshot.common_values[2] = feedback.small_wheel;

    for (std::size_t i = 0; i < kRobotForceSensorCount; ++i) {
        snapshot.common_values[3 + i] = feedback.force_sensor[i];
    }
    snapshot.common_values[13] = feedback.lifter;
    snapshot.common_values[14] = feedback.deliver_force;
    snapshot.common_values[15] = feedback.rotate_degree;
    snapshot.common_values[16] = feedback.follow_force;

    for (std::size_t i = 0; i < kRobotPublishedAxisCount; ++i) {
        snapshot.common_values[17 + i] = feedback.axes_pos[i];
    }
}

} // namespace device::beckhoff
