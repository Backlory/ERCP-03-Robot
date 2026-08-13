#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include "beckhoff_snapshot.hpp"

namespace device::beckhoff {

inline void MarkSnapshotGroup(BeckhoffSnapshot &snapshot,
                              SnapshotGroup group,
                              std::size_t sample_index,
                              bool any_read_succeeded,
                              std::uint64_t sampled_at_unix_ns)
{
    const auto bit = static_cast<std::uint8_t>(group);
    if (any_read_succeeded) {
        snapshot.valid_groups |= bit;
        snapshot.stale_groups &= static_cast<std::uint8_t>(~bit);
        snapshot.sampled_at_unix_ns[sample_index] = sampled_at_unix_ns;
    } else {
        snapshot.stale_groups |= bit;
    }
}

/**
 * @brief 功能：清空不可用的 ERCP 可选状态组及其诊断标志。
 * @details 机制：重置 ERCP 字段、有效/过期位和采样时间，不影响公共 Beckhoff 状态组。
 */
/**
 * @brief 功能：清空不可用的 ERCP 可选状态组及其诊断标志。
 * @details 机制：重置 ERCP 字段、有效/过期位和采样时间，不影响公共 Beckhoff 状态组。
 */
inline void ClearOptionalErcpGroups(BeckhoffSnapshot &snapshot)
{
    constexpr auto groups =
        static_cast<std::uint8_t>(SnapshotErcpState | SnapshotErcpFeedback);
    snapshot.ercp_state_ads_error = 0;
    snapshot.ercp_feedback_ads_error = 0;
    snapshot.ercp_flags = 0;
    snapshot.ercp_drive_errors = 0;
    snapshot.ercp_motor_errors = 0;
    snapshot.valid_groups &= static_cast<std::uint8_t>(~groups);
    snapshot.stale_groups &= static_cast<std::uint8_t>(~groups);
    snapshot.sampled_at_unix_ns[2] = 0;
    snapshot.sampled_at_unix_ns[3] = 0;
}

/**
 * @brief 功能：完成一次 Beckhoff 轮询的时间戳和连接质量收尾。
 * @details 机制：无 ADS 错误时清零连续失败计数并标记 Running；有错误时饱和递增计数并标记 Degraded。
 */
inline void FinalizeSnapshotPoll(BeckhoffSnapshot &snapshot,
                                 std::uint64_t completed_at_unix_ns)
{
    snapshot.poll_completed_unix_ns = completed_at_unix_ns;
    snapshot.published_unix_ns = completed_at_unix_ns;
    if (snapshot.overall_ads_error == 0) {
        snapshot.consecutive_failed_polls = 0;
        snapshot.connection_state = SnapshotConnectionState::Running;
        return;
    }

    if (snapshot.consecutive_failed_polls != (std::numeric_limits<std::uint32_t>::max)())
        snapshot.consecutive_failed_polls += 1;
    snapshot.connection_state = SnapshotConnectionState::Degraded;
}

} // namespace device::beckhoff
