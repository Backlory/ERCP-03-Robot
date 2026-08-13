#include <iostream>
#include <chrono>

#include <Poco/Net/DatagramSocket.h>

#include "robot_settings.hpp"
#include "YunSBot.h"

namespace ercp {

/**
 * @brief 初始化一个 Robot V3 控制通道的状态发送端和命令接收端。
 * @details 根据 loopback 配置建立状态目标、设置对端过滤规则，并绑定控制 UDP 服务及统计初始时间。
 */
YunSBot::_control_channel::_control_channel(YunSBot &p,
                                            protocol::v3::Source channel_source,
                                            std::string remote_address,
                                            std::uint16_t status_port,
                                            std::uint16_t control_port,
                                            bool loopback_only)
    : parent(p)
    , source(channel_source)
    , receiver(channel_source)
    , client()
{
    if (loopback_only)
        remote_address = "127.0.0.1";

    const Poco::Net::SocketAddress statusAddress(remote_address, status_port);
    client.connect(statusAddress);
    ROBOT_INFO(GetSettings().Basic.Verbose() > 0,
               fmt::format("Robot V3 status target {}", statusAddress.toString()))

    // M4: 31002 (non-loopback) only accepts datagrams from the configured peer
    // (basic.master); 31004 stays loopback-bound and needs no source filter.
    // Deployment precondition: the Master's actual egress IP must match the
    // configured address (NAT or other source rewriting is unsupported).
    filter_peer_ = !loopback_only;
    expected_peer_ = statusAddress.host();

    const Poco::Net::SocketAddress controlAddress(loopback_only ? "127.0.0.1" : "0.0.0.0",
                                                  control_port);
    handlers.push_back(this);
    server = std::make_shared<UdpServer>(handlers, controlAddress);
    last_stats_log_ = std::chrono::steady_clock::now();
    ROBOT_INFO(GetSettings().Basic.Verbose() > 0,
               fmt::format("Robot V3 control bind {}", server->address().toString()))
}

/**
 * @brief 功能：接收控制通道 UDP 数据并交给 Robot V3 命令解析器。
 * @details 机制：先按配置过滤对端地址，再把字节缓冲交给 CommandReceiver；解析结果和统计由接收器统一维护。
 */
void YunSBot::_control_channel::processData(char *buf)
{
    // M4: drop datagrams whose source IP differs from the configured peer
    // (see constructor comment); counted as rejected in receiver stats.
    if (filter_peer_) {
        const auto sender = Poco::Net::UDPHandler::address(buf);
        if (sender.host() != expected_peer_) {
            receiver.RecordRejected();
            const auto dropNow = std::chrono::steady_clock::now();
            if (dropNow - last_peer_drop_log_ >= std::chrono::seconds(10)) {
                ROBOT_ERROR(
                    GetSettings().Basic.Verbose() > 0,
                    fmt::format(
                        "Robot V3 control datagram dropped: unexpected peer {} (expected {})",
                        sender.toString(),
                        expected_peer_.toString()))
                last_peer_drop_log_ = dropNow;
            }
            return;
        }
    }

    const auto size = static_cast<std::size_t>(payloadSize(buf));
    const auto *bytes = reinterpret_cast<const std::uint8_t *>(payload(buf));

    std::string error;
    receiver.AcceptDatagram(bytes, size, &error);

    const auto now = std::chrono::steady_clock::now();
    if (GetSettings().Basic.Verbose() > 0 && now - last_stats_log_ >= std::chrono::seconds(60)) {
        const auto stats = receiver.Stats();
        ROBOT_INFO(
            true,
            fmt::format("Robot V3 command stats source={} received={} accepted={} rejected={} "
                        "duplicate={} out_of_order={} sequence_gaps={} max_consecutive_gap={} "
                        "session_restarts={} last_session={} last_sequence={}",
                        static_cast<std::uint16_t>(source),
                        stats.received,
                        stats.accepted,
                        stats.rejected,
                        stats.duplicate,
                        stats.out_of_order,
                        stats.gaps,
                        stats.max_consecutive_gap,
                        stats.restarts,
                        stats.last_session_id,
                        stats.last_sequence))
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

/**
 * @brief 功能：从控制通道读取仍在有效窗口内的最新命令。
 * @details 机制：转发到线程安全接收器，调用方同时获得命令元数据用于源仲裁和应用审计。
 */
bool YunSBot::_control_channel::GetCommand(protocol::v3::ControlPayload &command,
                                           robot_udp_v3::CommandMetadata &metadata,
                                           double overtime) const
{
    return receiver.TryGet(command, metadata, overtime);
}

bool YunSBot::_control_channel::LatestCommand(protocol::v3::ControlPayload &command,
                                              robot_udp_v3::CommandMetadata &metadata) const
{
    return receiver.Latest(command, metadata);
}

robot_udp_v3::ReceiveStats YunSBot::_control_channel::Stats() const
{
    return receiver.Stats();
}

int YunSBot::_control_channel::SendStatus(const protocol::v3::Bytes &data)
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
