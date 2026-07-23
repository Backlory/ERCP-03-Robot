#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "protocol/robot_udp_v2.hpp"
#include "robot_udp_v2_runtime.hpp"

namespace protocol = ercp::protocol::v2;
namespace runtime = ercp::robot_udp_v2;

namespace {

int failures = 0;

void Expect(bool condition, const char *message)
{
    if (condition) return;
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
}

protocol::ControlPayload SampleControl()
{
    protocol::ControlPayload payload;
    payload.values = { 1.0, -2.0, 3.5, -4.5, 5.25, -6.25, 7.0, -8.0, 9.0, -10.0 };
    payload.switches = 0x002D;
    return payload;
}

protocol::Bytes ControlPacket(protocol::Source source, std::uint64_t session,
    std::uint64_t sequence, const protocol::ControlPayload &payload = SampleControl())
{
    protocol::Header header;
    header.message_type = protocol::MessageType::RobotControl;
    header.source = source;
    header.session_id = session;
    header.sequence = sequence;
    header.sent_at_unix_ns = 123456789;

    protocol::Bytes bytes;
    std::string error;
    Expect(protocol::encodeControl(header, payload, bytes, &error), "control packet encodes");
    return bytes;
}

void TestCodec()
{
    const auto payload = SampleControl();
    auto bytes = ControlPacket(protocol::Source::Master, 0x0102030405060708ull,
        0x1112131415161718ull, payload);
    Expect(bytes.size() == 152, "control wire size is 152");
    Expect(bytes[0] == 0x45 && bytes[1] == 0x52 && bytes[2] == 0x43 && bytes[3] == 0x50,
        "control golden magic");
    Expect(bytes[10] == 0 && bytes[11] == 1 && bytes[12] == 0 && bytes[13] == 2,
        "control golden type and source");
    Expect(bytes[16] == 0 && bytes[17] == 0 && bytes[18] == 0 && bytes[19] == 104,
        "control golden payload size");
    Expect(bytes[48] == 0x3F && bytes[49] == 0xF0, "control golden binary64");

    protocol::Header decodedHeader;
    protocol::ControlPayload decoded;
    std::string error;
    Expect(protocol::decodeControl(bytes.data(), bytes.size(), decodedHeader, decoded, &error),
        "control round-trip decodes");
    Expect(decodedHeader.session_id == 0x0102030405060708ull
            && decodedHeader.sequence == 0x1112131415161718ull,
        "control header round-trip");
    Expect(decoded.values == payload.values && decoded.switches == payload.switches,
        "control payload round-trip");

    auto invalid = bytes;
    invalid[0] = 0;
    Expect(!protocol::decodeControl(invalid.data(), invalid.size(), decodedHeader, decoded, &error),
        "wrong magic rejected");
    invalid = bytes;
    invalid[5] = 3;
    Expect(!protocol::decodeControl(invalid.data(), invalid.size(), decodedHeader, decoded, &error),
        "wrong version rejected");
    Expect(!protocol::decodeControl(bytes.data(), bytes.size() - 1, decodedHeader, decoded, &error),
        "wrong size rejected");
    invalid = bytes;
    invalid[12] = 0;
    invalid[13] = 1;
    Expect(!protocol::decodeControl(invalid.data(), invalid.size(), decodedHeader, decoded, &error),
        "wrong control source rejected");
    invalid = bytes;
    invalid[44] = 1;
    Expect(!protocol::decodeControl(invalid.data(), invalid.size(), decodedHeader, decoded, &error),
        "non-zero header reserved rejected");
    invalid = bytes;
    invalid[130] = 1;
    Expect(!protocol::decodeControl(invalid.data(), invalid.size(), decodedHeader, decoded, &error),
        "non-zero control reserved rejected");

    protocol::Header header;
    header.message_type = protocol::MessageType::RobotControl;
    header.source = protocol::Source::Master;
    header.session_id = 1;
    auto nonFinite = payload;
    nonFinite.values[0] = (std::numeric_limits<double>::quiet_NaN)();
    protocol::Bytes output;
    Expect(!protocol::encodeControl(header, nonFinite, output, &error), "NaN rejected");
    nonFinite.values[0] = (std::numeric_limits<double>::infinity)();
    Expect(!protocol::encodeControl(header, nonFinite, output, &error), "infinity rejected");
}

void TestFullStatus()
{
    protocol::FullStatusPayload status;
    status.runtime.lifecycle = protocol::RobotLifecycle::Running;
    status.runtime.mode = protocol::RobotMode::Automatic;
    status.runtime.active_source = protocol::Source::Cloud;
    status.beckhoff_common.values[0] = 12.5;
    status.raw_io.encoders[4] = 42;
    status.ercp_feedback.values[11] = -3.25;
    status.applied_command.latest_write_attempt.command = SampleControl();
    status.applied_command.latest_write_attempt.source = protocol::Source::Cloud;
    status.applied_command.latest_write_attempt.result = protocol::ApplyResult::Succeeded;
    status.ads_diagnostics.snapshot_sequence = 99;
    status.sampled_at_unix_ns.fill(1000);

    protocol::Header header;
    header.message_type = protocol::MessageType::RobotStatus;
    header.source = protocol::Source::Robot;
    header.session_id = 7;
    header.sequence = 8;
    header.sent_at_unix_ns = 9;

    protocol::Bytes bytes;
    std::string error;
    Expect(protocol::encodeFullStatus(header, status, bytes, &error),
        "full status encodes without linking an ADS driver");
    Expect(bytes.size() == 1024, "full status wire size is 1024");
    Expect(bytes.size() <= protocol::kMaxPacketSize, "full status is within 1200-byte limit");

    protocol::Header decodedHeader;
    protocol::FullStatusPayload decoded;
    Expect(protocol::decodeFullStatus(bytes.data(), bytes.size(), decodedHeader, decoded, &error),
        "full status round-trip decodes");
    Expect(decodedHeader.sequence == 8 && decoded.ads_diagnostics.snapshot_sequence == 99,
        "full status metadata round-trip");
    Expect(decoded.beckhoff_common.values[0] == 12.5
            && decoded.raw_io.encoders[4] == 42
            && decoded.ercp_feedback.values[11] == -3.25,
        "full status business groups round-trip");

    auto overlappingErcpFlags = bytes;
    constexpr std::size_t kErcpFlagsOffset = 480;
    overlappingErcpFlags[kErcpFlagsOffset + 1] = 1u << 3;
    Expect(!protocol::decodeFullStatus(overlappingErcpFlags.data(),
               overlappingErcpFlags.size(), decodedHeader, decoded, &error),
        "overlapping ERCP total-error flags are rejected");

    status.beckhoff_common.move_state = static_cast<protocol::BeckhoffMoveState>(0xF1234567u);
    status.ercp_state.type = static_cast<protocol::ErcpDeviceType>(-101);
    status.ercp_state.move_status = static_cast<protocol::ErcpMoveState>(-102);
    status.ercp_feedback.inject_state_01 = static_cast<protocol::InjectorState>(-103);
    Expect(protocol::encodeFullStatus(header, status, bytes, &error)
            && protocol::decodeFullStatus(bytes.data(), bytes.size(), decodedHeader, decoded,
                &error)
            && static_cast<std::uint32_t>(decoded.beckhoff_common.move_state) == 0xF1234567u
            && static_cast<std::int32_t>(decoded.ercp_state.type) == -101
            && static_cast<std::int32_t>(decoded.ercp_state.move_status) == -102
            && static_cast<std::int32_t>(decoded.ercp_feedback.inject_state_01) == -103,
        "unregistered PLC enum values preserve their fixed-width raw representation");
}

void TestSequenceAndSessions()
{
    runtime::CommandReceiver receiver(protocol::Source::Master);
    auto packet = ControlPacket(protocol::Source::Master, 10, 1);
    Expect(receiver.Accept(packet.data(), packet.size()), "first sequence accepted");
    Expect(!receiver.Accept(packet.data(), packet.size()), "duplicate rejected");

    packet = ControlPacket(protocol::Source::Master, 10, 0);
    Expect(!receiver.Accept(packet.data(), packet.size()), "out-of-order rejected");
    packet = ControlPacket(protocol::Source::Master, 10, 4);
    Expect(receiver.Accept(packet.data(), packet.size()), "forward sequence accepted");

    packet = ControlPacket(protocol::Source::Master, 20, 0);
    Expect(receiver.Accept(packet.data(), packet.size()), "new session accepted as restart");
    packet = ControlPacket(protocol::Source::Master, 10, 5);
    Expect(!receiver.Accept(packet.data(), packet.size()), "retired session cannot become active again");

    for (std::uint64_t session = 21; session <= 52; ++session) {
        packet = ControlPacket(protocol::Source::Master, session, 0);
        Expect(receiver.Accept(packet.data(), packet.size()), "bounded retired-session history advances");
    }
    packet = ControlPacket(protocol::Source::Master, 20, 1);
    Expect(!receiver.Accept(packet.data(), packet.size()),
        "the most recent 32 retired sessions remain rejected");

    packet = ControlPacket(protocol::Source::Cloud, 30, 0);
    Expect(!receiver.Accept(packet.data(), packet.size()), "unexpected endpoint source rejected");

    const auto stats = receiver.Stats();
    Expect(stats.received == 40 && stats.accepted == 35 && stats.rejected == 5,
        "sequence counters are complete");
    Expect(stats.duplicate == 1 && stats.out_of_order == 1 && stats.gaps == 2
            && stats.restarts == 33,
        "sequence classifications are correct");
    Expect(stats.last_session_id == 52 && stats.last_sequence == 0,
        "retired packet does not change active session");
}

void TestV2OnlyAndFreshness()
{
    runtime::CommandReceiver receiver(protocol::Source::Cloud);
    std::string error;
    const std::vector<std::uint8_t> legacyBytes(152, 0);
    Expect(!receiver.AcceptDatagram(legacyBytes.data(), legacyBytes.size(), &error),
        "native-layout command packet is rejected");

    protocol::ControlPayload payload;
    runtime::CommandMetadata metadata;
    Expect(!receiver.TryGet(payload, metadata, 1.0), "rejected native packet is unavailable");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    Expect(!receiver.TryGet(payload, metadata, 0.001), "missing command is unavailable");

    auto v2 = ControlPacket(protocol::Source::Cloud, 50, 0);
    Expect(receiver.AcceptDatagram(v2.data(), v2.size(), &error),
        "V2 command is accepted");
    Expect(receiver.TryGet(payload, metadata, 1.0), "fresh V2 command is available");

    auto malformedV2 = v2;
    malformedV2[5] = 3;
    Expect(!receiver.AcceptDatagram(malformedV2.data(), malformedV2.size(), &error),
        "invalid V2 version is rejected");

    const std::uint8_t junk[3] = { 1, 2, 3 };
    const auto before = receiver.Stats();
    Expect(!receiver.AcceptDatagram(junk, sizeof(junk), &error),
        "unsupported datagram rejected");
    const auto after = receiver.Stats();
    Expect(after.received == before.received + 1 && after.rejected == before.rejected + 1,
        "unsupported datagram is counted");
}

void TestAppliedCommandTracking()
{
    runtime::AppliedCommandTracker tracker;
    runtime::CommandMetadata metadata;
    metadata.source = protocol::Source::Master;
    metadata.session_id = 1;
    metadata.sequence = 10;
    metadata.received_unix_ns = 100;

    const auto first = SampleControl();
    tracker.MarkAttempt(first, metadata, protocol::ApplyResult::Succeeded, 0, 200, true);

    auto failed = first;
    failed.values[1] = 77;
    metadata.sequence = 11;
    tracker.MarkAttempt(failed, metadata, protocol::ApplyResult::Failed, 0x701, 300, false);
    auto snapshot = tracker.Snapshot();
    Expect(snapshot.latest_write_attempt.command.values[1] == 77
            && snapshot.latest_write_attempt.ads_error == 0x701,
        "failed ADS write remains visible as latest attempt");
    Expect(snapshot.last_successful_write.command.values[1] == first.values[1]
            && snapshot.last_successful_write.command_sequence == 10,
        "failed ADS write does not replace last success");

    const auto zero = runtime::ZeroControl();
    metadata.sequence = 12;
    tracker.MarkAttempt(zero, metadata, protocol::ApplyResult::TimedOutToZero, 0, 400, true);
    snapshot = tracker.Snapshot();
    Expect(snapshot.latest_write_attempt.result == protocol::ApplyResult::TimedOutToZero
            && snapshot.last_successful_write.result == protocol::ApplyResult::TimedOutToZero,
        "successful timeout-to-zero retains timeout semantics");
    Expect(snapshot.last_successful_write.command.values == zero.values
            && snapshot.last_successful_write.command.switches == 0,
        "successful timeout writes an all-zero command");

    metadata.sequence = 13;
    tracker.MarkAttempt(zero, metadata, protocol::ApplyResult::Failed, 0x702, 500, false);
    snapshot = tracker.Snapshot();
    Expect(snapshot.latest_write_attempt.result == protocol::ApplyResult::Failed
            && snapshot.latest_write_attempt.ads_error == 0x702
            && snapshot.last_successful_write.command_sequence == 12,
        "failed safety-zero write is not reported as successfully timed out");
}

void TestControlSourceSelection()
{
    Expect(runtime::SelectedControlSource(false) == protocol::Source::Master,
        "manual mode selects the Master 31002 command source");
    Expect(runtime::SelectedControlSource(true) == protocol::Source::Cloud,
        "automatic mode selects the Cloud 31004 command source");
}

} // namespace

int main()
{
    TestCodec();
    TestFullStatus();
    TestSequenceAndSessions();
    TestV2OnlyAndFreshness();
    TestAppliedCommandTracking();
    TestControlSourceSelection();

    if (failures != 0) {
        std::cerr << failures << " Robot UDP V2 regression test(s) failed\n";
        return 1;
    }
    std::cout << "Robot UDP V2 regression tests passed\n";
    return 0;
}
