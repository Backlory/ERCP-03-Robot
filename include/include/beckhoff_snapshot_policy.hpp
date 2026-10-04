#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

#include "beckhoff_snapshot.hpp"

namespace device::beckhoff {

// 清空一个来源的全部反馈，保留独立的 ADS 错误诊断。
inline void ClearRobotFeedback(BeckhoffSnapshot &snapshot)
{
    snapshot.valid_sources &= static_cast<std::uint8_t>(~SnapshotRobot);
    snapshot.robot_feedback_acquired_unix_ns = 0;
    snapshot.move_state = 0;
    snapshot.output_switches = 0;
    snapshot.power_level = 0;
    snapshot.prepare_state = 0;
    snapshot.error_flags = 0;
    snapshot.drive_errors = 0;
    snapshot.motor_errors = 0;
    snapshot.scope_type = 0;
    snapshot.common_values.fill(0);
}

inline void ClearErcpFeedback(BeckhoffSnapshot &snapshot)
{
    snapshot.valid_sources &= static_cast<std::uint8_t>(~SnapshotErcp);
    snapshot.ercp_feedback_acquired_unix_ns = 0;
    snapshot.ercp_flags = 0;
    snapshot.ercp_drive_errors = 0;
    snapshot.ercp_motor_errors = 0;
    snapshot.ercp_type = 0;
    snapshot.ercp_move_status = 0;
    snapshot.ercp_deliver_force = 0;
    snapshot.guide_wire_force = 0;
    snapshot.bow_force = 0;
    snapshot.ercp_deliver_position = 0;
    snapshot.guide_wire_position = 0;
    snapshot.inject_current_position_01 = 0;
    snapshot.inject_current_position_02 = 0;
    snapshot.inject_state_01 = 0;
    snapshot.inject_state_02 = 0;
    snapshot.balloon_pressure = 0;
    snapshot.operator_position = 0;
}

// ERCP 状态和数值反馈必须同时读取成功。失败立即清空该来源。
inline void CompleteFeedbackRead(BeckhoffSnapshot &snapshot, SnapshotSource source,
                                 bool all_reads_succeeded, std::uint64_t acquired_unix_ns)
{
    if (!all_reads_succeeded) {
        if (source == SnapshotRobot) ClearRobotFeedback(snapshot);
        else ClearErcpFeedback(snapshot);
        return;
    }
    snapshot.valid_sources |= static_cast<std::uint8_t>(source);
    if (source == SnapshotRobot) snapshot.robot_feedback_acquired_unix_ns = acquired_unix_ns;
    else snapshot.ercp_feedback_acquired_unix_ns = acquired_unix_ns;
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
