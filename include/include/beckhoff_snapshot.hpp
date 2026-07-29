#pragma once

#include <array>
#include <cstdint>

namespace device::beckhoff {

enum class SnapshotConnectionState : std::uint8_t {
    Disconnected = 0,
    Connecting = 1,
    Running = 2,
    Degraded = 3,
};

enum SnapshotGroup : std::uint8_t {
    SnapshotCommon = 1u << 0,
    SnapshotRawIo = 1u << 1,
    SnapshotErcpState = 1u << 2,
    SnapshotErcpFeedback = 1u << 3,
};

struct GoldDiscreteCommand {
    int robot_action = -1;
    bool operate = false;
    bool cooperate = false;
    double handle_6d[6]{};
    bool buttons[3]{};
    double inject_velocity[2]{};
    double inject_position[2]{};
    bool inject_enable[2]{};
};

struct BeckhoffSnapshot {
    std::uint64_t sequence = 0;
    std::uint64_t poll_started_unix_ns = 0;
    std::uint64_t poll_completed_unix_ns = 0;
    std::uint64_t published_unix_ns = 0;
    SnapshotConnectionState connection_state = SnapshotConnectionState::Disconnected;
    std::uint8_t valid_groups = 0;
    std::uint8_t stale_groups = 0;
    std::uint32_t consecutive_failed_polls = 0;
    std::uint32_t overall_ads_error = 0;
    std::uint32_t common_ads_error = 0;
    std::uint32_t raw_io_ads_error = 0;
    std::uint32_t ercp_state_ads_error = 0;
    std::uint32_t ercp_feedback_ads_error = 0;
    std::uint32_t command_write_ads_error = 0;

    std::uint32_t move_state = 0;
    std::uint16_t output_switches = 0;
    std::int16_t power_level = 0;
    std::uint8_t prepare_state = 0;
    std::uint8_t error_flags = 0;
    std::uint32_t drive_errors = 0;
    std::uint32_t motor_errors = 0;
    std::int32_t scope_type = 0;
    std::array<double, 36> common_values{};

    std::array<std::int32_t, 5> encoders{};
    std::array<std::int16_t, 7> sensors{};

    std::uint16_t ercp_flags = 0;
    std::uint16_t ercp_drive_errors = 0;
    std::uint16_t ercp_motor_errors = 0;
    std::int32_t ercp_type = 0;
    std::int32_t ercp_move_status = 0;

    double ercp_deliver_force = 0;
    double guide_wire_force = 0;
    double bow_force = 0;
    double ercp_deliver_position = 0;
    double guide_wire_position = 0;
    double inject_current_position_01 = 0;
    double inject_current_position_02 = 0;
    std::int32_t inject_state_01 = 0;
    std::int32_t inject_state_02 = 0;
    std::int16_t balloon_pressure = 0;
    double operator_position = 0;

    // Common, raw IO, ERCP state, and ERCP feedback sample times.
    std::array<std::uint64_t, 4> sampled_at_unix_ns{};
};

} // namespace device::beckhoff
