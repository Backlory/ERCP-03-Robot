#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>

#include "robot_udp_v2_runtime.hpp"

namespace ercp::robot_udp_v2 {

std::uint64_t UnixNowNs()
{
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

std::uint64_t MakeSessionId()
{
    std::random_device random;
    std::uint64_t result = (static_cast<std::uint64_t>(random()) << 32) | random();
    if (result == 0) result = UnixNowNs() ^ 0x455243505632ull;
    return result == 0 ? 1 : result;
}

protocol::ControlPayload ZeroControl()
{
    return protocol::ControlPayload{};
}

protocol::Source SelectedControlSource(bool automatic_mode)
{
    return automatic_mode ? protocol::Source::Cloud : protocol::Source::Master;
}

CommandReceiver::CommandReceiver(protocol::Source expected_source)
    : expected_source_(expected_source)
{
}

void CommandReceiver::RetireSession(std::uint64_t session_id)
{
    constexpr std::size_t kRetiredSessionLimit = 32;
    if (session_id == 0 || !retired_session_ids_.insert(session_id).second) return;

    retired_session_order_.push_back(session_id);
    while (retired_session_order_.size() > kRetiredSessionLimit) {
        retired_session_ids_.erase(retired_session_order_.front());
        retired_session_order_.pop_front();
    }
}

void CommandReceiver::RecordRejected()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.received;
    ++stats_.rejected;
}

bool CommandReceiver::AcceptDatagram(const std::uint8_t *data, std::size_t size,
    std::string *error)
{
    return Accept(data, size, error);
}

bool CommandReceiver::Accept(const std::uint8_t *data, std::size_t size, std::string *error)
{
    protocol::Header header;
    protocol::ControlPayload payload;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.received;
    }

    if (!protocol::decodeControl(data, size, header, payload, error)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.rejected;
        return false;
    }
    if (header.source != expected_source_) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.rejected;
        if (error != nullptr) *error = "unexpected command source";
        return false;
    }

    const auto received = std::chrono::steady_clock::now();
    const auto received_unix_ns = UnixNowNs();
    std::lock_guard<std::mutex> lock(mutex_);

    if (retired_session_ids_.count(header.session_id) != 0) {
        ++stats_.rejected;
        if (error != nullptr) *error = "retired command session";
        return false;
    }

    if (stats_.last_session_id != 0 && stats_.last_session_id != header.session_id) {
        RetireSession(stats_.last_session_id);
        ++stats_.restarts;
        stats_.has_sequence = false;
    }
    stats_.last_session_id = header.session_id;
    if (stats_.has_sequence) {
        if (header.sequence == stats_.last_sequence) {
            ++stats_.duplicate;
            ++stats_.rejected;
            return false;
        }
        if (header.sequence < stats_.last_sequence) {
            ++stats_.out_of_order;
            ++stats_.rejected;
            return false;
        }
        const auto gap = header.sequence - stats_.last_sequence - 1;
        if (gap != 0) {
            const auto available = (std::numeric_limits<std::uint64_t>::max)() - stats_.gaps;
            stats_.gaps += std::min(gap, available);
            stats_.max_consecutive_gap = std::max(stats_.max_consecutive_gap, gap);
        }
    }

    latest_payload_ = payload;
    latest_metadata_ = { header.source, header.session_id, header.sequence, received_unix_ns };
    latest_received_ = received;
    has_latest_ = true;
    stats_.last_sequence = header.sequence;
    stats_.has_sequence = true;
    ++stats_.accepted;
    return true;
}

bool CommandReceiver::TryGet(protocol::ControlPayload &payload, CommandMetadata &metadata,
    double max_age_seconds) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_ || max_age_seconds < 0) return false;
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now()
        - latest_received_).count();
    if (age > max_age_seconds) return false;
    payload = latest_payload_;
    metadata = latest_metadata_;
    return true;
}

bool CommandReceiver::Latest(protocol::ControlPayload &payload, CommandMetadata &metadata) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_) return false;
    payload = latest_payload_;
    metadata = latest_metadata_;
    return true;
}

bool CommandReceiver::IsOnline(double max_age_seconds) const
{
    protocol::ControlPayload payload;
    CommandMetadata metadata;
    return TryGet(payload, metadata, max_age_seconds);
}

ReceiveStats CommandReceiver::Stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void AppliedCommandTracker::MarkAttempt(const protocol::ControlPayload &command,
    const CommandMetadata &metadata, protocol::ApplyResult result, std::uint32_t ads_error,
    std::uint64_t applied_unix_ns, bool write_succeeded)
{
    protocol::AppliedCommandRecord record;
    record.command = command;
    record.source = metadata.source;
    record.result = result;
    record.command_session_id = metadata.session_id;
    record.command_sequence = metadata.sequence;
    record.received_unix_ns = metadata.received_unix_ns;
    record.applied_unix_ns = applied_unix_ns;
    record.ads_error = ads_error;

    std::lock_guard<std::mutex> lock(mutex_);
    payload_.latest_write_attempt = record;
    if (write_succeeded) {
        payload_.last_successful_write = record;
    }
}

protocol::AppliedCommandPayload AppliedCommandTracker::Snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return payload_;
}

} // namespace ercp::robot_udp_v2
