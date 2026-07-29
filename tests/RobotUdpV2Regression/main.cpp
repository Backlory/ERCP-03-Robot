#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
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
    Expect(bytes.size() == 224, "control wire size is 224");
    Expect(bytes[0] == 0x45 && bytes[1] == 0x52 && bytes[2] == 0x43 && bytes[3] == 0x50,
        "control golden magic");
    Expect(bytes[10] == 0 && bytes[11] == 1 && bytes[12] == 0 && bytes[13] == 2,
        "control golden type and source");
    Expect(bytes[16] == 0 && bytes[17] == 0 && bytes[18] == 0 && bytes[19] == 176,
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
    invalid[5] = 2;
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
    invalid[218] = 1;
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
    status.ercp_feedback.operator_position = 0.75;
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
    Expect(bytes.size() == 1200, "full status wire size is 1200");
    Expect(bytes.size() <= protocol::kMaxPacketSize, "full status is within 1200-byte limit");

    protocol::Header decodedHeader;
    protocol::FullStatusPayload decoded;
    Expect(protocol::decodeFullStatus(bytes.data(), bytes.size(), decodedHeader, decoded, &error),
        "full status round-trip decodes");
    Expect(decodedHeader.sequence == 8 && decoded.ads_diagnostics.snapshot_sequence == 99,
        "full status metadata round-trip");
    Expect(decoded.beckhoff_common.values[0] == 12.5
            && decoded.ercp_feedback.operator_position == 0.75,
        "full status business groups round-trip");

    auto overlappingErcpFlags = bytes;
    constexpr std::size_t kErcpFlagsOffset = 496;
    overlappingErcpFlags[kErcpFlagsOffset + 1] = 1u << 5;
    Expect(!protocol::decodeFullStatus(overlappingErcpFlags.data(),
               overlappingErcpFlags.size(), decodedHeader, decoded, &error),
        "unknown ERCP flags are rejected");

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

// ===================== shared-wire 黄金字节测试 =====================
// 固定输入约定与 shared-wire/golden_gen/golden_gen.cpp 逐字面一致(fix_plan.md §3.5)。

std::uint64_t PU(std::size_t offset) { return offset % 251; }
double PF(std::size_t offset) { return static_cast<double>(offset % 251) + 0.5; }

// 读取 golden hex fixture(大写连续十六进制文本, 忽略空白)。
// 依次探测: ERCP_GOLDEN_DIR 环境变量、若干相对候选路径(测试目录 golden 副本)。
protocol::Bytes LoadGoldenHex(const std::string &name, std::string &usedPath)
{
    std::vector<std::string> candidates;
#ifdef _MSC_VER
    char *envDir = nullptr;
    std::size_t envLen = 0;
    if (_dupenv_s(&envDir, &envLen, "ERCP_GOLDEN_DIR") == 0 && envDir != nullptr) {
        candidates.push_back(std::string(envDir) + "/" + name);
        std::free(envDir);
    }
#else
    if (const char *dir = std::getenv("ERCP_GOLDEN_DIR")) {
        candidates.push_back(std::string(dir) + "/" + name);
    }
#endif
    std::string prefix;
    for (int depth = 0; depth < 7; ++depth) {
        candidates.push_back(prefix + "golden/" + name);
        candidates.push_back(prefix + "tests/RobotUdpV2Regression/golden/" + name);
        candidates.push_back(prefix + "03-Robot/tests/RobotUdpV2Regression/golden/" + name);
        prefix += "../";
    }
    for (const auto &candidate : candidates) {
        std::ifstream file(candidate, std::ios::binary);
        if (!file) continue;
        protocol::Bytes bytes;
        int hi = -1;
        char c = 0;
        while (file.get(c)) {
            int digit = -1;
            if (c >= '0' && c <= '9') digit = c - '0';
            else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
            else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
            else continue;
            if (hi < 0) { hi = digit; }
            else { bytes.push_back(static_cast<std::uint8_t>((hi << 4) | digit)); hi = -1; }
        }
        usedPath = candidate;
        return bytes;
    }
    usedPath.clear();
    return {};
}

protocol::Header GoldenControlHeader()
{
    protocol::Header header;
    header.message_type = protocol::MessageType::RobotControl;
    header.source = protocol::Source::Master;
    header.session_id = 0x1122334455667788ull;
    header.sequence = 42;
    header.sent_at_unix_ns = 0x0102030405060708ull;
    return header;
}

protocol::ControlPayload GoldenControlPayload()
{
    protocol::ControlPayload payload;
    for (std::size_t i = 0; i < payload.values.size(); ++i) {
        payload.values[i] = static_cast<double>(i) + 0.5;
    }
    payload.switches = 0x002A;
    payload.ercp_switches = 0x0015;
    payload.robot_action = 3;
    for (std::size_t i = 0; i < payload.ercp_6d.size(); ++i) payload.ercp_6d[i] = i + 10.5;
    payload.inject_velocity = { 0.1, 0.5 };
    payload.inject_position = { 0.25, 0.75 };
    payload.inject_enables = 1;
    return payload;
}

protocol::Header GoldenStatusHeader()
{
    protocol::Header header;
    header.message_type = protocol::MessageType::RobotStatus;
    header.source = protocol::Source::Robot;
    header.session_id = 0x1122334455667788ull;
    header.sequence = 42;
    header.sent_at_unix_ns = 0x0102030405060708ull;
    return header;
}

protocol::FullStatusPayload GoldenStatusPayload()
{
    protocol::FullStatusPayload s;
    s.runtime.lifecycle = protocol::RobotLifecycle::Running;
    s.runtime.mode = protocol::RobotMode::Automatic;
    s.runtime.active_source = protocol::Source::Master;
    s.runtime.flags = 0x00000005;
    s.runtime.lifecycle_changed_unix_ns = PU(80);
    s.runtime.accepted_command_received_unix_ns = PU(88);
    s.beckhoff_common.move_state = protocol::BeckhoffMoveState::Following;
    s.beckhoff_common.output_switches = 0x0005;
    s.beckhoff_common.power_level = static_cast<std::int16_t>(PU(118));
    s.beckhoff_common.prepare_state = 1;
    s.beckhoff_common.error_flags = 1;
    s.beckhoff_common.drive_errors = 0x200001;
    s.beckhoff_common.motor_errors = 0x40001;
    s.beckhoff_common.scope_type = 2;
    for (std::size_t j = 0; j < 36; ++j) s.beckhoff_common.values[j] = PF(120 + 8 * j);
    s.ercp_state.flags = 0x0003;
    s.ercp_state.drive_errors = static_cast<std::uint16_t>(PU(484));
    s.ercp_state.motor_errors = static_cast<std::uint16_t>(PU(486));
    s.ercp_state.type = protocol::ErcpDeviceType::StoneBasket;
    s.ercp_state.move_status = protocol::ErcpMoveState::Opened;
    s.ercp_feedback.ercp_deliver_force = 1.25;
    s.ercp_feedback.guide_wire_force = 2.25;
    s.ercp_feedback.bow_force = 3.25;
    s.ercp_feedback.ercp_deliver_position = 4.25;
    s.ercp_feedback.guide_wire_position = 5.25;
    s.ercp_feedback.inject_current_position_01 = 0.25;
    s.ercp_feedback.inject_current_position_02 = 0.75;
    s.ercp_feedback.inject_state_01 = protocol::InjectorState::Injecting;
    s.ercp_feedback.inject_state_02 = protocol::InjectorState::Completed;
    s.ercp_feedback.balloon_pressure = 123;
    s.ercp_feedback.operator_position = 0.625;
    auto &r0 = s.applied_command.latest_write_attempt;
    for (std::size_t j = 0; j < 10; ++j) r0.command.values[j] = PF(640 + 8 * j);
    r0.command.switches = 0x002A;
    r0.source = protocol::Source::Master;
    r0.result = protocol::ApplyResult::Succeeded;
    r0.command_session_id = PU(728);
    r0.command_sequence = PU(736);
    r0.received_unix_ns = PU(744);
    r0.applied_unix_ns = PU(752);
    r0.ads_error = static_cast<std::uint32_t>(PU(760));
    auto &r1 = s.applied_command.last_successful_write;
    for (std::size_t j = 0; j < 10; ++j) r1.command.values[j] = PF(768 + 8 * j);
    r1.command.switches = 0x0015;
    r1.source = protocol::Source::Cloud;
    r1.result = protocol::ApplyResult::TimedOutToZero;
    r1.command_session_id = PU(856);
    r1.command_sequence = PU(864);
    r1.received_unix_ns = PU(872);
    r1.applied_unix_ns = PU(880);
    r1.ads_error = static_cast<std::uint32_t>(PU(888));
    s.ads_diagnostics.snapshot_sequence = PU(912);
    s.ads_diagnostics.poll_started_unix_ns = PU(920);
    s.ads_diagnostics.poll_completed_unix_ns = PU(928);
    s.ads_diagnostics.snapshot_published_unix_ns = PU(936);
    s.ads_diagnostics.connection_state = protocol::AdsConnectionState::Running;
    s.ads_diagnostics.valid_groups = 0x0D;
    s.ads_diagnostics.stale_groups = 0x01;
    s.ads_diagnostics.consecutive_failed_polls = static_cast<std::uint32_t>(PU(948));
    s.ads_diagnostics.overall_ads_error = static_cast<std::uint32_t>(PU(952));
    s.ads_diagnostics.common_ads_error = static_cast<std::uint32_t>(PU(956));
    s.ads_diagnostics.reserved_ads_error = 0;
    s.ads_diagnostics.ercp_state_ads_error = static_cast<std::uint32_t>(PU(964));
    s.ads_diagnostics.ercp_feedback_ads_error = static_cast<std::uint32_t>(PU(968));
    s.ads_diagnostics.command_write_ads_error = static_cast<std::uint32_t>(PU(972));
    s.sampled_at_unix_ns = { PU(64), PU(104), PU(416), PU(472),
                             PU(512), PU(632), PU(904), PU(984) };
    return s;
}

void ExpectBytesEqual(const protocol::Bytes &actual, const protocol::Bytes &golden,
    const char *what)
{
    if (actual.size() != golden.size()) {
        ++failures;
        std::cerr << "FAIL: " << what << " size " << actual.size()
                  << " != golden " << golden.size() << '\n';
        return;
    }
    for (std::size_t i = 0; i < actual.size(); ++i) {
        if (actual[i] != golden[i]) {
            ++failures;
            std::cerr << "FAIL: " << what << " first byte mismatch at offset " << i
                      << " (actual 0x" << std::hex << int(actual[i])
                      << " golden 0x" << int(golden[i]) << std::dec << ")\n";
            return;
        }
    }
}

void TestGoldenControlFixture()
{
    std::string path;
    const auto golden = LoadGoldenHex("robot_control_224.hex", path);
    Expect(!golden.empty(), "robot_control_224.hex fixture found");
    if (golden.empty()) return;
    std::cout << "golden control fixture: " << path << '\n';
    Expect(golden.size() == 224, "control golden is 224 bytes");

    protocol::Bytes encoded;
    std::string error;
    Expect(protocol::encodeControl(GoldenControlHeader(), GoldenControlPayload(), encoded, &error),
        "golden control input encodes");
    ExpectBytesEqual(encoded, golden, "encode(control fixture) == golden hex");

    protocol::Header header;
    protocol::ControlPayload payload;
    Expect(protocol::decodeControl(golden.data(), golden.size(), header, payload, &error),
        "golden control hex decodes");
    Expect(header.session_id == 0x1122334455667788ull && header.sequence == 42
            && header.sent_at_unix_ns == 0x0102030405060708ull
            && header.source == protocol::Source::Master,
        "golden control header fields match the fixed input");
    bool valuesMatch = payload.switches == 0x002A;
    for (std::size_t i = 0; i < payload.values.size(); ++i) {
        valuesMatch = valuesMatch && payload.values[i] == static_cast<double>(i) + 0.5;
    }
    Expect(valuesMatch, "golden control payload fields match the fixed input");
}

void TestGoldenStatusFixture()
{
    std::string path;
    const auto golden = LoadGoldenHex("robot_status_1200.hex", path);
    Expect(!golden.empty(), "robot_status_1200.hex fixture found");
    if (golden.empty()) return;
    std::cout << "golden status fixture: " << path << '\n';
    Expect(golden.size() == 1200, "status golden is 1200 bytes");

    const auto fixture = GoldenStatusPayload();
    protocol::Bytes encoded;
    std::string error;
    Expect(protocol::encodeFullStatus(GoldenStatusHeader(), fixture, encoded, &error),
        "golden status input encodes");
    ExpectBytesEqual(encoded, golden, "encode(status fixture) == golden hex");

    protocol::Header header;
    protocol::FullStatusPayload decoded;
    Expect(protocol::decodeFullStatus(golden.data(), golden.size(), header, decoded, &error),
        "golden status hex decodes");
    Expect(header.session_id == 0x1122334455667788ull && header.sequence == 42
            && header.sent_at_unix_ns == 0x0102030405060708ull
            && header.source == protocol::Source::Robot,
        "golden status header fields match the fixed input");
    Expect(decoded.runtime.lifecycle == fixture.runtime.lifecycle
            && decoded.runtime.mode == fixture.runtime.mode
            && decoded.runtime.active_source == fixture.runtime.active_source
            && decoded.runtime.flags == fixture.runtime.flags
            && decoded.runtime.lifecycle_changed_unix_ns
                == fixture.runtime.lifecycle_changed_unix_ns
            && decoded.runtime.accepted_command_received_unix_ns
                == fixture.runtime.accepted_command_received_unix_ns,
        "golden status runtime group matches the fixed input");
    Expect(decoded.beckhoff_common.move_state == fixture.beckhoff_common.move_state
            && decoded.beckhoff_common.output_switches == fixture.beckhoff_common.output_switches
            && decoded.beckhoff_common.power_level == fixture.beckhoff_common.power_level
            && decoded.beckhoff_common.values == fixture.beckhoff_common.values,
        "golden status Beckhoff group matches the fixed input");
    Expect(decoded.ercp_state.flags == fixture.ercp_state.flags
            && decoded.ercp_state.drive_errors == fixture.ercp_state.drive_errors
            && decoded.ercp_state.motor_errors == fixture.ercp_state.motor_errors
            && decoded.ercp_state.type == fixture.ercp_state.type
            && decoded.ercp_state.move_status == fixture.ercp_state.move_status,
        "golden status ERCP state group matches the fixed input");
    Expect(decoded.ercp_feedback.bow_force == fixture.ercp_feedback.bow_force
            && decoded.ercp_feedback.operator_position == fixture.ercp_feedback.operator_position
            && decoded.ercp_feedback.inject_state_01 == fixture.ercp_feedback.inject_state_01
            && decoded.ercp_feedback.inject_state_02 == fixture.ercp_feedback.inject_state_02,
        "golden status ERCP feedback group matches the fixed input");
    const auto recordEqual = [](const protocol::AppliedCommandRecord &a,
        const protocol::AppliedCommandRecord &b) {
        return a.command.values == b.command.values && a.command.switches == b.command.switches
            && a.source == b.source && a.result == b.result
            && a.command_session_id == b.command_session_id
            && a.command_sequence == b.command_sequence
            && a.received_unix_ns == b.received_unix_ns
            && a.applied_unix_ns == b.applied_unix_ns && a.ads_error == b.ads_error;
    };
    Expect(recordEqual(decoded.applied_command.latest_write_attempt,
               fixture.applied_command.latest_write_attempt)
            && recordEqual(decoded.applied_command.last_successful_write,
                fixture.applied_command.last_successful_write),
        "golden status applied command group matches the fixed input");
    Expect(decoded.ads_diagnostics.snapshot_sequence == fixture.ads_diagnostics.snapshot_sequence
            && decoded.ads_diagnostics.poll_started_unix_ns
                == fixture.ads_diagnostics.poll_started_unix_ns
            && decoded.ads_diagnostics.poll_completed_unix_ns
                == fixture.ads_diagnostics.poll_completed_unix_ns
            && decoded.ads_diagnostics.snapshot_published_unix_ns
                == fixture.ads_diagnostics.snapshot_published_unix_ns
            && decoded.ads_diagnostics.connection_state
                == fixture.ads_diagnostics.connection_state
            && decoded.ads_diagnostics.valid_groups == fixture.ads_diagnostics.valid_groups
            && decoded.ads_diagnostics.stale_groups == fixture.ads_diagnostics.stale_groups
            && decoded.ads_diagnostics.consecutive_failed_polls
                == fixture.ads_diagnostics.consecutive_failed_polls
            && decoded.ads_diagnostics.overall_ads_error
                == fixture.ads_diagnostics.overall_ads_error
            && decoded.ads_diagnostics.common_ads_error
                == fixture.ads_diagnostics.common_ads_error
            && decoded.ads_diagnostics.reserved_ads_error
                == fixture.ads_diagnostics.reserved_ads_error
            && decoded.ads_diagnostics.ercp_state_ads_error
                == fixture.ads_diagnostics.ercp_state_ads_error
            && decoded.ads_diagnostics.ercp_feedback_ads_error
                == fixture.ads_diagnostics.ercp_feedback_ads_error
            && decoded.ads_diagnostics.command_write_ads_error
                == fixture.ads_diagnostics.command_write_ads_error,
        "golden status ADS diagnostics group matches the fixed input");
    Expect(decoded.sampled_at_unix_ns == fixture.sampled_at_unix_ns,
        "golden status sampled_at timestamps match the fixed input");
}

} // namespace

int main()
{
    std::cout << "kRobotUdpV2SyncVersion = " << protocol::kRobotUdpV2SyncVersion << '\n';

    TestCodec();
    TestFullStatus();
    TestSequenceAndSessions();
    TestV2OnlyAndFreshness();
    TestAppliedCommandTracking();
    TestControlSourceSelection();
    TestGoldenControlFixture();
    TestGoldenStatusFixture();

    if (failures != 0) {
        std::cerr << failures << " Robot UDP V2 regression test(s) failed\n";
        return 1;
    }
    std::cout << "Robot UDP V2 regression tests passed\n";
    return 0;
}
