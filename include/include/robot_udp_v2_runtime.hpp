#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>

#include "protocol/robot_udp_v2.hpp"

namespace ercp::robot_udp_v2 {

namespace protocol = ercp::protocol::v2;

struct CommandMetadata {
    protocol::Source source = static_cast<protocol::Source>(0);
    std::uint64_t session_id = 0;
    std::uint64_t sequence = 0;
    std::uint64_t received_unix_ns = 0;
};

struct ReceiveStats {
    std::uint64_t received = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t duplicate = 0;
    std::uint64_t out_of_order = 0;
    std::uint64_t gaps = 0;
    std::uint64_t max_consecutive_gap = 0;
    std::uint64_t restarts = 0;
    std::uint64_t last_sequence = 0;
    std::uint64_t last_session_id = 0;
    bool has_sequence = false;
};

class CommandReceiver {
public:
    explicit CommandReceiver(protocol::Source expected_source);

    bool AcceptDatagram(const std::uint8_t *data, std::size_t size,
        std::string *error = nullptr);
    bool Accept(const std::uint8_t *data, std::size_t size, std::string *error = nullptr);
    void RecordRejected();

    bool TryGet(protocol::ControlPayload &payload, CommandMetadata &metadata,
        double max_age_seconds) const;
    bool Latest(protocol::ControlPayload &payload, CommandMetadata &metadata) const;
    bool IsOnline(double max_age_seconds) const;

    protocol::Source ExpectedSource() const { return expected_source_; }
    ReceiveStats Stats() const;

private:
    void RetireSession(std::uint64_t session_id);

    protocol::Source expected_source_;
    mutable std::mutex mutex_;
    protocol::ControlPayload latest_payload_;
    CommandMetadata latest_metadata_;
    std::chrono::steady_clock::time_point latest_received_;
    bool has_latest_ = false;
    ReceiveStats stats_;
    std::deque<std::uint64_t> retired_session_order_;
    std::set<std::uint64_t> retired_session_ids_;
};

class AppliedCommandTracker {
public:
    AppliedCommandTracker() = default;

    void MarkAttempt(const protocol::ControlPayload &payload, const CommandMetadata &metadata,
        protocol::ApplyResult result, std::uint32_t ads_error, std::uint64_t applied_unix_ns,
        bool write_succeeded);

    protocol::AppliedCommandPayload Snapshot() const;

private:
    mutable std::mutex mutex_;
    protocol::AppliedCommandPayload payload_;
};

std::uint64_t UnixNowNs();
std::uint64_t MakeSessionId();
protocol::ControlPayload ZeroControl();
protocol::Source SelectedControlSource(bool automatic_mode);

} // namespace ercp::robot_udp_v2
