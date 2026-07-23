#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace ercp::protocol::v2 {

using Bytes = std::vector<std::uint8_t>;

constexpr std::uint32_t kMagic = 0x45524350u; // "ERCP"
constexpr std::uint16_t kVersionMajor = 2;
constexpr std::uint16_t kVersionMinor = 0;
constexpr std::size_t kHeaderSize = 48;
constexpr std::size_t kControlPayloadSize = 104;
constexpr std::size_t kControlPacketSize = kHeaderSize + kControlPayloadSize;
constexpr std::size_t kStatusDirectorySize = 8;
constexpr std::size_t kStatusGroupHeaderSize = 16;
constexpr std::size_t kMaxPacketSize = 1200;

enum class MessageType : std::uint16_t {
    RobotControl = 1,
    RobotStatus = 2,
};

enum class Source : std::uint16_t {
    Robot = 1,
    Master = 2,
    Cloud = 3,
    Test = 255,
};

enum class GroupId : std::uint16_t {
    RobotRuntime = 1,
    BeckhoffCommon = 2,
    RawIo = 3,
    ErcpState = 4,
    ErcpFeedback = 5,
    AppliedCommand = 6,
    AdsDiagnostics = 7,
    Extension = 8,
};

struct Header {
    std::uint32_t magic = kMagic;
    std::uint16_t version_major = kVersionMajor;
    std::uint16_t version_minor = kVersionMinor;
    std::uint16_t header_size = static_cast<std::uint16_t>(kHeaderSize);
    MessageType message_type = MessageType::RobotControl;
    Source source = Source::Test;
    std::uint16_t flags = 0;
    std::uint32_t payload_size = 0;
    std::uint64_t session_id = 1;
    std::uint64_t sequence = 0;
    std::uint64_t sent_at_unix_ns = 0;
    std::uint32_t reserved = 0;
};

struct ControlPayload {
    std::array<double, 10> values{};
    std::uint16_t switches = 0;
};

struct StatusGroup {
    std::uint16_t id = 0;
    std::uint16_t version = 1;
    std::uint64_t sampled_at_unix_ns = 0;
    Bytes payload;
};

struct StatusMessage {
    Header header;
    std::vector<StatusGroup> groups;
};

enum class RobotLifecycle : std::uint8_t {
    Unknown = 0,
    Stopped = 1,
    Starting = 2,
    Running = 3,
    Stopping = 4,
    Fault = 5,
};

enum class RobotMode : std::uint8_t {
    Unknown = 0,
    Manual = 1,
    Automatic = 2,
};

enum class BeckhoffMoveState : std::uint32_t {
    Initial = 0,
    Folding = 10,
    Folded = 11,
    Opening = 20,
    Opened = 21,
    Following = 30,
    Followed = 31,
};

enum class ErcpDeviceType : std::int32_t {
    Cutter = 0,
    Basket = 1,
    Balloon = 2,
    DrainageTube = 3,
};

enum class ErcpMoveState : std::int32_t {
    Idle = 0,
    Folding = 10,
    Folded = 11,
    Opening = 20,
    Opened = 21,
    Following = 30,
    Followed = 31,
    Loading = 40,
    Loaded = 41,
    Exchanging = 50,
    Exchanged = 51,
};

enum class InjectorState : std::int32_t {
    Idle = 0,
    Injecting = 10,
    Completed = 11,
};

enum class ApplyResult : std::uint16_t {
    None = 0,
    Attempted = 1,
    Succeeded = 2,
    Failed = 3,
    Rejected = 4,
    TimedOutToZero = 5,
};

enum class AdsConnectionState : std::uint8_t {
    Disconnected = 0,
    Connecting = 1,
    Running = 2,
    Degraded = 3,
};

struct RobotRuntimePayload {
    RobotLifecycle lifecycle = RobotLifecycle::Unknown;
    RobotMode mode = RobotMode::Unknown;
    Source active_source = static_cast<Source>(0);
    std::uint32_t flags = 0;
    std::uint64_t lifecycle_changed_unix_ns = 0;
    std::uint64_t accepted_command_received_unix_ns = 0;
};

struct BeckhoffCommonPayload {
    BeckhoffMoveState move_state = BeckhoffMoveState::Initial;
    std::uint16_t output_switches = 0;
    std::int16_t power_level = 0;
    std::array<double, 36> values{};
};

struct RawIoPayload {
    std::array<std::int32_t, 5> encoders{};
    std::array<std::int16_t, 7> sensors{};
};

struct ErcpStatePayload {
    std::uint16_t flags = 0;
    std::uint16_t drive_errors = 0;
    std::uint16_t motor_errors = 0;
    ErcpDeviceType type = ErcpDeviceType::Cutter;
    ErcpMoveState move_status = ErcpMoveState::Idle;
};

struct ErcpFeedbackPayload {
    std::array<double, 12> values{};
    InjectorState inject_state_01 = InjectorState::Idle;
    InjectorState inject_state_02 = InjectorState::Idle;
};

struct AppliedCommandRecord {
    ControlPayload command;
    Source source = static_cast<Source>(0);
    ApplyResult result = ApplyResult::None;
    std::uint64_t command_session_id = 0;
    std::uint64_t command_sequence = 0;
    std::uint64_t received_unix_ns = 0;
    std::uint64_t applied_unix_ns = 0;
    std::uint32_t ads_error = 0;
};

struct AppliedCommandPayload {
    AppliedCommandRecord latest_write_attempt;
    AppliedCommandRecord last_successful_write;
};

struct AdsDiagnosticsPayload {
    std::uint64_t snapshot_sequence = 0;
    std::uint64_t poll_started_unix_ns = 0;
    std::uint64_t poll_completed_unix_ns = 0;
    std::uint64_t snapshot_published_unix_ns = 0;
    AdsConnectionState connection_state = AdsConnectionState::Disconnected;
    std::uint8_t valid_groups = 0;
    std::uint8_t stale_groups = 0;
    std::uint32_t consecutive_failed_polls = 0;
    std::uint32_t overall_ads_error = 0;
    std::uint32_t common_ads_error = 0;
    std::uint32_t raw_io_ads_error = 0;
    std::uint32_t ercp_state_ads_error = 0;
    std::uint32_t ercp_feedback_ads_error = 0;
    std::uint32_t command_write_ads_error = 0;
};

struct FullStatusPayload {
    RobotRuntimePayload runtime;
    BeckhoffCommonPayload beckhoff_common;
    RawIoPayload raw_io;
    ErcpStatePayload ercp_state;
    ErcpFeedbackPayload ercp_feedback;
    AppliedCommandPayload applied_command;
    AdsDiagnosticsPayload ads_diagnostics;
    std::array<std::uint64_t, 8> sampled_at_unix_ns{};
};

namespace detail {

inline bool fail(std::string *error, const char *message)
{
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

class Writer {
public:
    explicit Writer(Bytes &bytes) : bytes_(bytes) {}

    void u8(std::uint8_t value) { bytes_.push_back(value); }

    void u16(std::uint16_t value)
    {
        bytes_.push_back(static_cast<std::uint8_t>(value >> 8));
        bytes_.push_back(static_cast<std::uint8_t>(value));
    }

    void u32(std::uint32_t value)
    {
        bytes_.push_back(static_cast<std::uint8_t>(value >> 24));
        bytes_.push_back(static_cast<std::uint8_t>(value >> 16));
        bytes_.push_back(static_cast<std::uint8_t>(value >> 8));
        bytes_.push_back(static_cast<std::uint8_t>(value));
    }

    void u64(std::uint64_t value)
    {
        for (int shift = 56; shift >= 0; shift -= 8) {
            bytes_.push_back(static_cast<std::uint8_t>(value >> shift));
        }
    }

    void i16(std::int16_t value) { u16(static_cast<std::uint16_t>(value)); }
    void i32(std::int32_t value) { u32(static_cast<std::uint32_t>(value)); }

    void f64(double value)
    {
        std::uint64_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value), "binary64 must be 64 bits");
        std::memcpy(&bits, &value, sizeof(bits));
        u64(bits);
    }

    void bytes(const std::uint8_t *data, std::size_t size)
    {
        if (size != 0) bytes_.insert(bytes_.end(), data, data + size);
    }

private:
    Bytes &bytes_;
};

class Reader {
public:
    Reader(const std::uint8_t *data, std::size_t size) : data_(data), size_(size) {}

    bool u8(std::uint8_t &value)
    {
        if (!require(1)) return false;
        value = data_[offset_++];
        return true;
    }

    bool u16(std::uint16_t &value)
    {
        if (!require(2)) return false;
        value = (static_cast<std::uint16_t>(data_[offset_]) << 8)
            | static_cast<std::uint16_t>(data_[offset_ + 1]);
        offset_ += 2;
        return true;
    }

    bool u32(std::uint32_t &value)
    {
        if (!require(4)) return false;
        value = (static_cast<std::uint32_t>(data_[offset_]) << 24)
            | (static_cast<std::uint32_t>(data_[offset_ + 1]) << 16)
            | (static_cast<std::uint32_t>(data_[offset_ + 2]) << 8)
            | static_cast<std::uint32_t>(data_[offset_ + 3]);
        offset_ += 4;
        return true;
    }

    bool u64(std::uint64_t &value)
    {
        if (!require(8)) return false;
        value = 0;
        for (int i = 0; i < 8; ++i) {
            value = (value << 8) | data_[offset_ + i];
        }
        offset_ += 8;
        return true;
    }

    bool i16(std::int16_t &value)
    {
        std::uint16_t bits = 0;
        if (!u16(bits)) return false;
        value = static_cast<std::int16_t>(bits);
        return true;
    }

    bool i32(std::int32_t &value)
    {
        std::uint32_t bits = 0;
        if (!u32(bits)) return false;
        value = static_cast<std::int32_t>(bits);
        return true;
    }

    bool f64(double &value)
    {
        std::uint64_t bits = 0;
        if (!u64(bits)) return false;
        std::memcpy(&value, &bits, sizeof(value));
        return std::isfinite(value);
    }

    bool copy(Bytes &value, std::size_t size)
    {
        if (!require(size)) return false;
        value.assign(data_ + offset_, data_ + offset_ + size);
        offset_ += size;
        return true;
    }

    std::size_t remaining() const { return size_ - offset_; }

private:
    bool require(std::size_t count) const { return count <= remaining(); }

    const std::uint8_t *data_;
    std::size_t size_;
    std::size_t offset_ = 0;
};

inline bool validSourceFor(MessageType type, Source source)
{
    if (source == Source::Test) return true;
    if (type == MessageType::RobotControl) {
        return source == Source::Master || source == Source::Cloud;
    }
    return source == Source::Robot;
}

inline bool validHeader(const Header &header, MessageType expected, std::string *error)
{
    if (header.magic != kMagic) return fail(error, "invalid magic");
    if (header.version_major != kVersionMajor || header.version_minor != kVersionMinor) {
        return fail(error, "unsupported protocol version");
    }
    if (header.header_size != kHeaderSize) return fail(error, "invalid header size");
    if (header.message_type != expected) return fail(error, "unexpected message type");
    if (!validSourceFor(expected, header.source)) return fail(error, "invalid source");
    if (header.flags != 0 || header.reserved != 0) return fail(error, "non-zero header reserved data");
    if (header.session_id == 0) return fail(error, "session id is zero");
    return true;
}

inline bool validSwitches(std::uint16_t switches, std::string *error)
{
    return (switches & static_cast<std::uint16_t>(~0x003Fu)) == 0
        ? true
        : fail(error, "unknown control switch bit");
}

inline std::uint16_t be16(const Bytes &bytes, std::size_t offset)
{
    return static_cast<std::uint16_t>((bytes[offset] << 8) | bytes[offset + 1]);
}

inline std::uint32_t be32(const Bytes &bytes, std::size_t offset)
{
    return (static_cast<std::uint32_t>(bytes[offset]) << 24)
        | (static_cast<std::uint32_t>(bytes[offset + 1]) << 16)
        | (static_cast<std::uint32_t>(bytes[offset + 2]) << 8)
        | static_cast<std::uint32_t>(bytes[offset + 3]);
}

inline bool allZero(const Bytes &bytes, std::size_t offset, std::size_t size)
{
    for (std::size_t i = offset; i < offset + size; ++i) {
        if (bytes[i] != 0) return false;
    }
    return true;
}

inline bool finiteF64(const Bytes &bytes, std::size_t offset)
{
    std::uint64_t bits = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        bits = (bits << 8) | bytes[offset + i];
    }
    double value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return std::isfinite(value);
}

inline std::size_t knownGroupSize(std::uint16_t id)
{
    switch (static_cast<GroupId>(id)) {
    case GroupId::RobotRuntime: return 24;
    case GroupId::BeckhoffCommon: return 296;
    case GroupId::RawIo: return 40;
    case GroupId::ErcpState: return 24;
    case GroupId::ErcpFeedback: return 104;
    case GroupId::AppliedCommand: return 256;
    case GroupId::AdsDiagnostics: return 64;
    case GroupId::Extension: return 32;
    default: return 0;
    }
}

inline bool validKnownGroup(const StatusGroup &group, std::string *error)
{
    const std::size_t expected = knownGroupSize(group.id);
    if (expected == 0) return true;
    if (group.version != 1 || group.payload.size() != expected) {
        return fail(error, "known status group version or size mismatch");
    }

    switch (static_cast<GroupId>(group.id)) {
    case GroupId::RobotRuntime:
        {
        const std::uint16_t activeSource = be16(group.payload, 2);
        if (group.payload[0] > 5 || group.payload[1] > 2) {
            return fail(error, "invalid robot runtime enum");
        }
        if (activeSource != 0 && activeSource != 1 && activeSource != 2
            && activeSource != 3 && activeSource != 255) {
            return fail(error, "invalid active source");
        }
        if ((be32(group.payload, 4) & ~0x0Fu) != 0) {
            return fail(error, "unknown robot runtime flag");
        }
        return true;
        }
    case GroupId::BeckhoffCommon:
        if ((be16(group.payload, 4) & ~0x0007u) != 0) {
            return fail(error, "unknown Beckhoff output bit");
        }
        for (std::size_t offset = 8; offset < group.payload.size(); offset += 8) {
            if (!finiteF64(group.payload, offset)) {
                return fail(error, "non-finite Beckhoff feedback");
            }
        }
        return true;
    case GroupId::RawIo:
        return allZero(group.payload, 34, 6)
            ? true
            : fail(error, "non-zero raw IO reserved data");
    case GroupId::ErcpState:
        if ((be16(group.payload, 0) & ~0x0007u) != 0 || be16(group.payload, 2) != 0
            || (be16(group.payload, 4) & ~0x3FFFu) != 0
            || (be16(group.payload, 6) & ~0x0FFFu) != 0
            || !allZero(group.payload, 16, 8)) {
            return fail(error, "invalid ERCP state reserved data");
        }
        return true;
    case GroupId::ErcpFeedback:
        for (std::size_t offset = 0; offset < 96; offset += 8) {
            if (!finiteF64(group.payload, offset)) {
                return fail(error, "non-finite ERCP feedback");
            }
        }
        return true;
    case GroupId::AppliedCommand:
        for (std::size_t record = 0; record < 2; ++record) {
            const std::size_t base = record * 128;
            for (std::size_t i = 0; i < 10; ++i) {
                if (!finiteF64(group.payload, base + i * 8)) {
                    return fail(error, "non-finite applied command");
                }
            }
            const std::uint16_t source = be16(group.payload, base + 82);
            const std::uint16_t result = be16(group.payload, base + 84);
            if ((be16(group.payload, base + 80) & ~0x003Fu) != 0
                || (source != 0 && source != 1 && source != 2 && source != 3 && source != 255)
                || result > 5
                || !allZero(group.payload, base + 86, 2)
                || !allZero(group.payload, base + 124, 4)) {
                return fail(error, "invalid applied command record");
            }
        }
        return true;
    case GroupId::AdsDiagnostics:
        if (group.payload[32] > 3 || (group.payload[33] & ~0x0Fu) != 0
            || (group.payload[34] & ~0x0Fu) != 0 || group.payload[35] != 0) {
            return fail(error, "invalid ADS diagnostics flags");
        }
        return true;
    case GroupId::Extension:
        return allZero(group.payload, 0, group.payload.size())
            ? true
            : fail(error, "non-zero status extension");
    default:
        return true;
    }
}

inline bool readHeader(Reader &reader, Header &header, std::string *error)
{
    std::uint16_t message = 0;
    std::uint16_t source = 0;
    if (!reader.u32(header.magic) || !reader.u16(header.version_major)
        || !reader.u16(header.version_minor) || !reader.u16(header.header_size)
        || !reader.u16(message) || !reader.u16(source) || !reader.u16(header.flags)
        || !reader.u32(header.payload_size) || !reader.u64(header.session_id)
        || !reader.u64(header.sequence) || !reader.u64(header.sent_at_unix_ns)
        || !reader.u32(header.reserved)) {
        return fail(error, "truncated header");
    }
    header.message_type = static_cast<MessageType>(message);
    header.source = static_cast<Source>(source);
    return true;
}

inline void writeHeader(Writer &writer, const Header &header, std::uint32_t payloadSize)
{
    writer.u32(header.magic);
    writer.u16(header.version_major);
    writer.u16(header.version_minor);
    writer.u16(header.header_size);
    writer.u16(static_cast<std::uint16_t>(header.message_type));
    writer.u16(static_cast<std::uint16_t>(header.source));
    writer.u16(header.flags);
    writer.u32(payloadSize);
    writer.u64(header.session_id);
    writer.u64(header.sequence);
    writer.u64(header.sent_at_unix_ns);
    writer.u32(header.reserved);
}

} // namespace detail

inline bool encodeControl(const Header &header, const ControlPayload &payload, Bytes &output,
    std::string *error = nullptr)
{
    if (!detail::validHeader(header, MessageType::RobotControl, error)) return false;
    if (!detail::validSwitches(payload.switches, error)) return false;
    for (double value : payload.values) {
        if (!std::isfinite(value)) return detail::fail(error, "non-finite control value");
    }

    output.clear();
    output.reserve(kControlPacketSize);
    detail::Writer writer(output);
    detail::writeHeader(writer, header, static_cast<std::uint32_t>(kControlPayloadSize));
    for (double value : payload.values) writer.f64(value);
    writer.u16(payload.switches);
    for (int i = 0; i < 22; ++i) writer.u8(0);
    return output.size() == kControlPacketSize;
}

inline bool decodeControl(const std::uint8_t *data, std::size_t size, Header &header,
    ControlPayload &payload, std::string *error = nullptr)
{
    if (data == nullptr || size != kControlPacketSize) return detail::fail(error, "invalid control size");
    detail::Reader reader(data, size);
    if (!detail::readHeader(reader, header, error)) return false;
    if (!detail::validHeader(header, MessageType::RobotControl, error)
        || header.payload_size != kControlPayloadSize) {
        return detail::fail(error, "invalid control header");
    }
    for (double &value : payload.values) {
        if (!reader.f64(value)) return detail::fail(error, "invalid control double");
    }
    if (!reader.u16(payload.switches) || !detail::validSwitches(payload.switches, error)) return false;
    Bytes reserved;
    if (!reader.copy(reserved, 22) || !detail::allZero(reserved, 0, reserved.size())) {
        return detail::fail(error, "non-zero control reserved data");
    }
    return reader.remaining() == 0;
}

inline bool isFullStatus(const std::vector<StatusGroup> &groups)
{
    if (groups.size() != 8) return false;
    for (std::size_t i = 0; i < groups.size(); ++i) {
        if (groups[i].id != static_cast<std::uint16_t>(i + 1)
            || groups[i].version != 1
            || groups[i].payload.size() != detail::knownGroupSize(groups[i].id)) {
            return false;
        }
    }
    return true;
}

inline bool encodeStatus(const Header &header, const std::vector<StatusGroup> &groups, Bytes &output,
    std::string *error = nullptr)
{
    if (!detail::validHeader(header, MessageType::RobotStatus, error)) return false;
    if (groups.size() > 0xFFFFu) return detail::fail(error, "too many status groups");

    std::size_t payloadSize = kStatusDirectorySize;
    bool seen[9] = {};
    for (const auto &group : groups) {
        if (group.id <= 8) {
            if (seen[group.id]) return detail::fail(error, "duplicate known status group");
            seen[group.id] = true;
        }
        if (!detail::validKnownGroup(group, error)) return false;
        if (group.payload.size() > 0xFFFFFFFFu - kStatusGroupHeaderSize) {
            return detail::fail(error, "status group too large");
        }
        payloadSize += kStatusGroupHeaderSize + group.payload.size();
    }
    if (isFullStatus(groups) && payloadSize + kHeaderSize != 1024) {
        return detail::fail(error, "full status wire size changed");
    }
    if (payloadSize + kHeaderSize > kMaxPacketSize) return detail::fail(error, "status packet too large");

    output.clear();
    output.reserve(kHeaderSize + payloadSize);
    detail::Writer writer(output);
    detail::writeHeader(writer, header, static_cast<std::uint32_t>(payloadSize));
    writer.u16(static_cast<std::uint16_t>(groups.size()));
    writer.u16(0);
    writer.u32(0);
    for (const auto &group : groups) {
        writer.u16(group.id);
        writer.u16(group.version);
        writer.u32(static_cast<std::uint32_t>(group.payload.size()));
        writer.u64(group.sampled_at_unix_ns);
        writer.bytes(group.payload.data(), group.payload.size());
    }
    return output.size() == kHeaderSize + payloadSize;
}

inline bool encodeFullStatus(const Header &header, const std::vector<StatusGroup> &groups, Bytes &output,
    std::string *error = nullptr)
{
    if (!isFullStatus(groups)) return detail::fail(error, "full status groups are incomplete");
    return encodeStatus(header, groups, output, error);
}

inline bool decodeStatus(const std::uint8_t *data, std::size_t size, StatusMessage &message,
    std::string *error = nullptr)
{
    if (data == nullptr || size < kHeaderSize + kStatusDirectorySize || size > kMaxPacketSize) {
        return detail::fail(error, "invalid status size");
    }
    detail::Reader reader(data, size);
    if (!detail::readHeader(reader, message.header, error)) return false;
    if (!detail::validHeader(message.header, MessageType::RobotStatus, error)
        || message.header.payload_size != size - kHeaderSize) {
        return detail::fail(error, "invalid status header");
    }

    std::uint16_t groupCount = 0;
    std::uint16_t schemaFlags = 0;
    std::uint32_t reserved = 0;
    if (!reader.u16(groupCount) || !reader.u16(schemaFlags) || !reader.u32(reserved)
        || schemaFlags != 0 || reserved != 0) {
        return detail::fail(error, "invalid status directory");
    }

    message.groups.clear();
    bool seen[9] = {};
    for (std::uint16_t i = 0; i < groupCount; ++i) {
        StatusGroup group;
        std::uint32_t groupSize = 0;
        if (!reader.u16(group.id) || !reader.u16(group.version) || !reader.u32(groupSize)
            || !reader.u64(group.sampled_at_unix_ns) || groupSize > reader.remaining()) {
            return detail::fail(error, "truncated status group");
        }
        if (group.id <= 8) {
            if (seen[group.id]) return detail::fail(error, "duplicate known status group");
            seen[group.id] = true;
        }
        if (!reader.copy(group.payload, groupSize) || !detail::validKnownGroup(group, error)) return false;
        message.groups.push_back(std::move(group));
    }
    if (reader.remaining() != 0) return detail::fail(error, "status group count does not consume packet");
    return true;
}

inline bool decodeFullStatus(const std::uint8_t *data, std::size_t size, StatusMessage &message,
    std::string *error = nullptr)
{
    if (!decodeStatus(data, size, message, error)) return false;
    return isFullStatus(message.groups) ? true : detail::fail(error, "full status groups are incomplete");
}

namespace detail {

inline StatusGroup group(GroupId id, std::uint64_t sampledAt, Bytes payload)
{
    StatusGroup result;
    result.id = static_cast<std::uint16_t>(id);
    result.sampled_at_unix_ns = sampledAt;
    result.payload = std::move(payload);
    return result;
}

inline void writeAppliedCommand(Writer &writer, const AppliedCommandRecord &record)
{
    for (double value : record.command.values) writer.f64(value);
    writer.u16(record.command.switches);
    writer.u16(static_cast<std::uint16_t>(record.source));
    writer.u16(static_cast<std::uint16_t>(record.result));
    writer.u16(0);
    writer.u64(record.command_session_id);
    writer.u64(record.command_sequence);
    writer.u64(record.received_unix_ns);
    writer.u64(record.applied_unix_ns);
    writer.u32(record.ads_error);
    writer.u32(0);
}

inline bool readAppliedCommand(Reader &reader, AppliedCommandRecord &record)
{
    for (double &value : record.command.values) {
        if (!reader.f64(value)) return false;
    }
    std::uint16_t source = 0;
    std::uint16_t result = 0;
    std::uint16_t reserved16 = 0;
    std::uint32_t reserved32 = 0;
    if (!reader.u16(record.command.switches) || !reader.u16(source) || !reader.u16(result)
        || !reader.u16(reserved16) || !reader.u64(record.command_session_id)
        || !reader.u64(record.command_sequence) || !reader.u64(record.received_unix_ns)
        || !reader.u64(record.applied_unix_ns) || !reader.u32(record.ads_error)
        || !reader.u32(reserved32) || reserved16 != 0 || reserved32 != 0) {
        return false;
    }
    record.source = static_cast<Source>(source);
    record.result = static_cast<ApplyResult>(result);
    return true;
}

inline const StatusGroup *findGroup(const StatusMessage &message, GroupId id)
{
    const auto rawId = static_cast<std::uint16_t>(id);
    for (const auto &candidate : message.groups) {
        if (candidate.id == rawId) return &candidate;
    }
    return nullptr;
}

} // namespace detail

inline std::vector<StatusGroup> buildFullStatusGroups(const FullStatusPayload &status)
{
    std::vector<StatusGroup> groups;
    groups.reserve(8);

    Bytes bytes;
    bytes.reserve(296);
    {
        detail::Writer writer(bytes);
        writer.u8(static_cast<std::uint8_t>(status.runtime.lifecycle));
        writer.u8(static_cast<std::uint8_t>(status.runtime.mode));
        writer.u16(static_cast<std::uint16_t>(status.runtime.active_source));
        writer.u32(status.runtime.flags);
        writer.u64(status.runtime.lifecycle_changed_unix_ns);
        writer.u64(status.runtime.accepted_command_received_unix_ns);
    }
    groups.push_back(detail::group(GroupId::RobotRuntime, status.sampled_at_unix_ns[0], std::move(bytes)));

    bytes.clear();
    {
        detail::Writer writer(bytes);
        writer.u32(static_cast<std::uint32_t>(status.beckhoff_common.move_state));
        writer.u16(status.beckhoff_common.output_switches);
        writer.i16(status.beckhoff_common.power_level);
        for (double value : status.beckhoff_common.values) writer.f64(value);
    }
    groups.push_back(detail::group(GroupId::BeckhoffCommon, status.sampled_at_unix_ns[1], std::move(bytes)));

    bytes.clear();
    {
        detail::Writer writer(bytes);
        for (std::int32_t value : status.raw_io.encoders) writer.i32(value);
        for (std::int16_t value : status.raw_io.sensors) writer.i16(value);
        for (int i = 0; i < 6; ++i) writer.u8(0);
    }
    groups.push_back(detail::group(GroupId::RawIo, status.sampled_at_unix_ns[2], std::move(bytes)));

    bytes.clear();
    {
        detail::Writer writer(bytes);
        writer.u16(status.ercp_state.flags);
        writer.u16(0);
        writer.u16(status.ercp_state.drive_errors);
        writer.u16(status.ercp_state.motor_errors);
        writer.i32(static_cast<std::int32_t>(status.ercp_state.type));
        writer.i32(static_cast<std::int32_t>(status.ercp_state.move_status));
        writer.u64(0);
    }
    groups.push_back(detail::group(GroupId::ErcpState, status.sampled_at_unix_ns[3], std::move(bytes)));

    bytes.clear();
    {
        detail::Writer writer(bytes);
        for (double value : status.ercp_feedback.values) writer.f64(value);
        writer.i32(static_cast<std::int32_t>(status.ercp_feedback.inject_state_01));
        writer.i32(static_cast<std::int32_t>(status.ercp_feedback.inject_state_02));
    }
    groups.push_back(detail::group(GroupId::ErcpFeedback, status.sampled_at_unix_ns[4], std::move(bytes)));

    bytes.clear();
    {
        detail::Writer writer(bytes);
        detail::writeAppliedCommand(writer, status.applied_command.latest_write_attempt);
        detail::writeAppliedCommand(writer, status.applied_command.last_successful_write);
    }
    groups.push_back(detail::group(GroupId::AppliedCommand, status.sampled_at_unix_ns[5], std::move(bytes)));

    bytes.clear();
    {
        detail::Writer writer(bytes);
        writer.u64(status.ads_diagnostics.snapshot_sequence);
        writer.u64(status.ads_diagnostics.poll_started_unix_ns);
        writer.u64(status.ads_diagnostics.poll_completed_unix_ns);
        writer.u64(status.ads_diagnostics.snapshot_published_unix_ns);
        writer.u8(static_cast<std::uint8_t>(status.ads_diagnostics.connection_state));
        writer.u8(status.ads_diagnostics.valid_groups);
        writer.u8(status.ads_diagnostics.stale_groups);
        writer.u8(0);
        writer.u32(status.ads_diagnostics.consecutive_failed_polls);
        writer.u32(status.ads_diagnostics.overall_ads_error);
        writer.u32(status.ads_diagnostics.common_ads_error);
        writer.u32(status.ads_diagnostics.raw_io_ads_error);
        writer.u32(status.ads_diagnostics.ercp_state_ads_error);
        writer.u32(status.ads_diagnostics.ercp_feedback_ads_error);
        writer.u32(status.ads_diagnostics.command_write_ads_error);
    }
    groups.push_back(detail::group(GroupId::AdsDiagnostics, status.sampled_at_unix_ns[6], std::move(bytes)));

    bytes.assign(32, 0);
    groups.push_back(detail::group(GroupId::Extension, status.sampled_at_unix_ns[7], std::move(bytes)));
    return groups;
}

inline bool encodeFullStatus(const Header &header, const FullStatusPayload &status, Bytes &output,
    std::string *error = nullptr)
{
    return encodeFullStatus(header, buildFullStatusGroups(status), output, error);
}

inline bool parseFullStatusPayload(const StatusMessage &message, FullStatusPayload &status,
    std::string *error = nullptr)
{
    if (!isFullStatus(message.groups)) return detail::fail(error, "full status groups are incomplete");
    FullStatusPayload parsed;

    const auto parseGroup = [&](GroupId id, auto &&parse) {
        const StatusGroup *group = detail::findGroup(message, id);
        if (group == nullptr) return false;
        parsed.sampled_at_unix_ns[static_cast<std::size_t>(id) - 1] = group->sampled_at_unix_ns;
        detail::Reader reader(group->payload.data(), group->payload.size());
        return parse(reader) && reader.remaining() == 0;
    };

    if (!parseGroup(GroupId::RobotRuntime, [&](detail::Reader &reader) {
            std::uint8_t lifecycle = 0;
            std::uint8_t mode = 0;
            std::uint16_t source = 0;
            if (!reader.u8(lifecycle) || !reader.u8(mode) || !reader.u16(source)
                || !reader.u32(parsed.runtime.flags)
                || !reader.u64(parsed.runtime.lifecycle_changed_unix_ns)
                || !reader.u64(parsed.runtime.accepted_command_received_unix_ns)) return false;
            parsed.runtime.lifecycle = static_cast<RobotLifecycle>(lifecycle);
            parsed.runtime.mode = static_cast<RobotMode>(mode);
            parsed.runtime.active_source = static_cast<Source>(source);
            return true;
        })) return detail::fail(error, "cannot parse robot runtime group");

    if (!parseGroup(GroupId::BeckhoffCommon, [&](detail::Reader &reader) {
            std::uint32_t moveState = 0;
            if (!reader.u32(moveState)
                || !reader.u16(parsed.beckhoff_common.output_switches)
                || !reader.i16(parsed.beckhoff_common.power_level)) return false;
            parsed.beckhoff_common.move_state = static_cast<BeckhoffMoveState>(moveState);
            for (double &value : parsed.beckhoff_common.values) if (!reader.f64(value)) return false;
            return true;
        })) return detail::fail(error, "cannot parse Beckhoff common group");

    if (!parseGroup(GroupId::RawIo, [&](detail::Reader &reader) {
            for (std::int32_t &value : parsed.raw_io.encoders) if (!reader.i32(value)) return false;
            for (std::int16_t &value : parsed.raw_io.sensors) if (!reader.i16(value)) return false;
            Bytes reserved;
            return reader.copy(reserved, 6);
        })) return detail::fail(error, "cannot parse raw IO group");

    if (!parseGroup(GroupId::ErcpState, [&](detail::Reader &reader) {
            std::uint16_t reserved16 = 0;
            std::uint64_t reserved64 = 0;
            std::int32_t type = 0;
            std::int32_t moveStatus = 0;
            if (!reader.u16(parsed.ercp_state.flags) || !reader.u16(reserved16)
                || !reader.u16(parsed.ercp_state.drive_errors)
                || !reader.u16(parsed.ercp_state.motor_errors) || !reader.i32(type)
                || !reader.i32(moveStatus) || !reader.u64(reserved64)) return false;
            parsed.ercp_state.type = static_cast<ErcpDeviceType>(type);
            parsed.ercp_state.move_status = static_cast<ErcpMoveState>(moveStatus);
            return true;
        })) return detail::fail(error, "cannot parse ERCP state group");

    if (!parseGroup(GroupId::ErcpFeedback, [&](detail::Reader &reader) {
            for (double &value : parsed.ercp_feedback.values) if (!reader.f64(value)) return false;
            std::int32_t injectState01 = 0;
            std::int32_t injectState02 = 0;
            if (!reader.i32(injectState01) || !reader.i32(injectState02)) return false;
            parsed.ercp_feedback.inject_state_01 = static_cast<InjectorState>(injectState01);
            parsed.ercp_feedback.inject_state_02 = static_cast<InjectorState>(injectState02);
            return true;
        })) return detail::fail(error, "cannot parse ERCP feedback group");

    if (!parseGroup(GroupId::AppliedCommand, [&](detail::Reader &reader) {
            return detail::readAppliedCommand(reader, parsed.applied_command.latest_write_attempt)
                && detail::readAppliedCommand(reader, parsed.applied_command.last_successful_write);
        })) return detail::fail(error, "cannot parse applied command group");

    if (!parseGroup(GroupId::AdsDiagnostics, [&](detail::Reader &reader) {
            std::uint8_t connection = 0;
            std::uint8_t reserved = 0;
            if (!reader.u64(parsed.ads_diagnostics.snapshot_sequence)
                || !reader.u64(parsed.ads_diagnostics.poll_started_unix_ns)
                || !reader.u64(parsed.ads_diagnostics.poll_completed_unix_ns)
                || !reader.u64(parsed.ads_diagnostics.snapshot_published_unix_ns)
                || !reader.u8(connection) || !reader.u8(parsed.ads_diagnostics.valid_groups)
                || !reader.u8(parsed.ads_diagnostics.stale_groups) || !reader.u8(reserved)
                || !reader.u32(parsed.ads_diagnostics.consecutive_failed_polls)
                || !reader.u32(parsed.ads_diagnostics.overall_ads_error)
                || !reader.u32(parsed.ads_diagnostics.common_ads_error)
                || !reader.u32(parsed.ads_diagnostics.raw_io_ads_error)
                || !reader.u32(parsed.ads_diagnostics.ercp_state_ads_error)
                || !reader.u32(parsed.ads_diagnostics.ercp_feedback_ads_error)
                || !reader.u32(parsed.ads_diagnostics.command_write_ads_error)) return false;
            parsed.ads_diagnostics.connection_state = static_cast<AdsConnectionState>(connection);
            return true;
        })) return detail::fail(error, "cannot parse ADS diagnostics group");

    if (!parseGroup(GroupId::Extension, [](detail::Reader &reader) {
            Bytes reserved;
            return reader.copy(reserved, 32);
        })) return detail::fail(error, "cannot parse extension group");

    status = std::move(parsed);
    return true;
}

inline bool decodeFullStatus(const std::uint8_t *data, std::size_t size, Header &header,
    FullStatusPayload &status, std::string *error = nullptr)
{
    StatusMessage message;
    if (!decodeFullStatus(data, size, message, error)
        || !parseFullStatusPayload(message, status, error)) return false;
    header = message.header;
    return true;
}

} // namespace ercp::protocol::v2
