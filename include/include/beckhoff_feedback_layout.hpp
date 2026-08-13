#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "beckhoff_snapshot.hpp"

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

// One error slot per static leaf request. A zero slot means that the leaf is
// available for this poll; a non-zero slot means that only that leaf is
// unavailable. The array is intentionally local to the read/apply seam and
// is not added to the UDP status model.
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

inline bool FeedbackLeafAvailable(const RobotFeedbackLeafErrors &errors, std::size_t index)
{
    return errors[index] == 0;
}

/**
 * @brief 将 Beckhoff 反馈叶字段按独立读取结果映射到统一状态快照。
 * @details 每个字段只在对应 ADS 读取成功时写入；失败字段清零，避免把旧值误当成当前有效反馈。
 */
// Apply only values whose corresponding leaf read succeeded. Failed leaves
// are cleared in the local snapshot and represented by common_ads_error plus
// the rate-limited field log in the ADS adapter; no startup policy is applied
// here.
inline void ApplyRobotFeedback(const RobotFeedbackLeaves &feedback,
                               const RobotFeedbackLeafErrors &errors,
                               BeckhoffSnapshot &snapshot)
{
    // 阶段一：把每个叶字段的错误码转换为可用性判断；阶段二：按 PLC/状态快照索引映射开关、连续值和轴位置。
    const auto available = [&](std::size_t index) {
        return FeedbackLeafAvailable(errors, index);
    };
    const auto applyDouble = [&](double &target, double value, std::size_t index) {
        target = available(index) ? value : 0;
    };

    snapshot.output_switches = 0;
    if (available(kFeedbackSwitchWaterIndex) && feedback.switch_water)
        snapshot.output_switches |= static_cast<std::uint16_t>(1u << 0);
    if (available(kFeedbackSwitchGasIndex) && feedback.switch_gas)
        snapshot.output_switches |= static_cast<std::uint16_t>(1u << 1);
    if (available(kFeedbackSwitchSuckIndex) && feedback.switch_suck)
        snapshot.output_switches |= static_cast<std::uint16_t>(1u << 2);

    snapshot.power_level = available(kFeedbackPowerLevelIndex) ? feedback.power_level : 0;
    applyDouble(snapshot.common_values[0], feedback.follow_length, kFeedbackFollowLengthIndex);
    applyDouble(snapshot.common_values[1], feedback.big_wheel, kFeedbackBigWheelIndex);
    applyDouble(snapshot.common_values[2], feedback.small_wheel, kFeedbackSmallWheelIndex);

    for (std::size_t i = 0; i < kRobotForceSensorCount; ++i) {
        applyDouble(snapshot.common_values[3 + i],
                    feedback.force_sensor[i],
                    kFeedbackForceSensorBaseIndex + i);
    }
    applyDouble(snapshot.common_values[13], feedback.lifter, kFeedbackLifterIndex);
    applyDouble(snapshot.common_values[14], feedback.deliver_force, kFeedbackDeliverForceIndex);
    applyDouble(snapshot.common_values[15], feedback.rotate_degree, kFeedbackRotateDegreeIndex);
    applyDouble(snapshot.common_values[16], feedback.follow_force, kFeedbackFollowForceIndex);

    for (std::size_t i = 0; i < kRobotPublishedAxisCount; ++i) {
        applyDouble(snapshot.common_values[17 + i],
                    feedback.axes_pos[i],
                    kFeedbackAxesBaseIndex + i);
    }
}

} // namespace device::beckhoff
