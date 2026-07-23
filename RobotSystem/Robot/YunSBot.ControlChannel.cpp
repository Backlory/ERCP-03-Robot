#include <iostream>
#include <chrono>

#include <Poco/Net/DatagramSocket.h>

#include "robot_settings.hpp"
#include "YunSBot.h"

namespace ercp {

YunSBot::_control_channel::_control_channel(YunSBot &p, protocol::v2::Source channel_source,
    std::string remote_address, std::uint16_t status_port, std::uint16_t control_port,
    bool loopback_only)
    : parent(p)
    , source(channel_source)
    , receiver(channel_source)
    , client()
{
    if (loopback_only) remote_address = "127.0.0.1";

    const Poco::Net::SocketAddress statusAddress(remote_address, status_port);
    client.connect(statusAddress);
    ROBOT_INFO(GetSettings().Basic.Verbose() > 0,
        fmt::format("Robot V2 status target {}", statusAddress.toString()))

    const Poco::Net::SocketAddress controlAddress(
        loopback_only ? "127.0.0.1" : "0.0.0.0", control_port);
    handlers.push_back(this);
    server = std::make_shared<UdpServer>(handlers, controlAddress);
    last_stats_log_ = std::chrono::steady_clock::now();
    ROBOT_INFO(GetSettings().Basic.Verbose() > 0,
        fmt::format("Robot V2 control bind {}", server->address().toString()))
}

void YunSBot::_control_channel::processData(char *buf)
{
    const auto size = static_cast<std::size_t>(payloadSize(buf));
    const auto *bytes = reinterpret_cast<const std::uint8_t *>(payload(buf));

    std::string error;
    receiver.AcceptDatagram(bytes, size, &error);

    const auto now = std::chrono::steady_clock::now();
    if (GetSettings().Basic.Verbose() > 0
        && now - last_stats_log_ >= std::chrono::seconds(60)) {
        const auto stats = receiver.Stats();
        ROBOT_INFO(true, fmt::format(
            "Robot V2 command stats source={} received={} accepted={} rejected={} "
            "duplicate={} out_of_order={} sequence_gaps={} max_consecutive_gap={} "
            "session_restarts={} last_session={} last_sequence={}",
            static_cast<std::uint16_t>(source), stats.received, stats.accepted, stats.rejected,
            stats.duplicate, stats.out_of_order, stats.gaps, stats.max_consecutive_gap,
            stats.restarts, stats.last_session_id, stats.last_sequence))
        last_stats_log_ = now;
    }
}

void YunSBot::_control_channel::processError(char *buf)
{
    ROBOT_ERROR(GetSettings().Basic.Verbose() > 0, Poco::Net::UDPHandler::error(buf))
}

bool YunSBot::_control_channel::IsOnline(double overtime) const
{
    return receiver.IsOnline(overtime);
}

bool YunSBot::_control_channel::GetCommand(protocol::v2::ControlPayload &command,
    robot_udp_v2::CommandMetadata &metadata, double overtime) const
{
    return receiver.TryGet(command, metadata, overtime);
}

bool YunSBot::_control_channel::LatestCommand(protocol::v2::ControlPayload &command,
    robot_udp_v2::CommandMetadata &metadata) const
{
    return receiver.Latest(command, metadata);
}

robot_udp_v2::ReceiveStats YunSBot::_control_channel::Stats() const
{
    return receiver.Stats();
}

int YunSBot::_control_channel::SendStatus(const protocol::v2::Bytes &data)
{
    try {
        return client.sendBytes(const_cast<std::uint8_t *>(data.data()),
            static_cast<int>(data.size()));
    } catch (const std::exception &exception) {
        ROBOT_ERROR(GetSettings().Basic.Verbose() > 0, exception.what())
        return 0;
    }
}

} // namespace ercp
