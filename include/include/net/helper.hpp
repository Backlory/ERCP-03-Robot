// [SHARED-WIRE] 本文件是跨工程 wire 协议定义,禁止在本工程内单独修改。
// SYNC-SOURCE : <repo-root>/shared-wire/net_helper.hpp
// SYNC-VERSION: 1
// SYNC-RULE   : 先改权威源并更新 golden/*.hex,再整文件复制到所有副本工程,最后各端跑黄金测试。
/**
 * @file net_helper.hpp
 * @brief Provides address parsing and small helpers shared by network adapters.
 */
#pragma once
#include <regex>
#include <boost/asio.hpp>

namespace net {

    // 同步版本号:与文件头 SYNC-VERSION 保持一致;黄金测试将其打印进输出,供人工比对各端副本版本。
    constexpr int kNetHelperSyncVersion = 1;

    struct _input_net_type {
    };

    struct _output_net_type {
    };

    namespace helper {

        /**
         * @brief 功能：解析 IPv4 或 IPv4:port 文本。
         * @details 机制：用正则提取地址和可选端口；无端口时返回 -1，格式不匹配时返回 false。
         */
        static bool parse_ip(const std::string &source, std::string &addr, int &port)
        {
            // 解析 IPv4 或 IPv4:port 文本；无端口时保留 -1，格式不匹配时不写入有效地址。
            port = -1;
            static const std::regex ip_regex("^((?:[0-9]{1,3}\\.){3}[0-9]{1,3})(?::([0-9]+))?$");
            std::smatch base_match;
            if (std::regex_match(source, base_match, ip_regex)) {
                if (base_match.size() >= 2)
                    addr = base_match[1].str();
                if (base_match.size() >= 3 && base_match[2].length())
                    port = std::stoi(base_match[2].str());
                return true;
            }
            return false;
        }

        using namespace boost::asio;
        using tcp = boost::asio::ip::tcp;
        using udp = boost::asio::ip::udp;

        template <typename protocol>
        static bool is_port_engaged(unsigned short port);

        template <>
        static bool is_port_engaged<tcp>(unsigned short port)
        {
            io_service svc;
            tcp::acceptor accp(svc);
            boost::system::error_code ec;
            accp.open(tcp::v4(), ec) || accp.bind({ tcp::v4(), port }, ec);
            return ec == error::address_in_use;
        }

        template <>
        static bool is_port_engaged<udp>(unsigned short port)
        {
            io_service svc;
            udp::socket soc(svc);
            boost::system::error_code ec;
            soc.open(udp::v4(), ec) || soc.bind({ udp::v4(), port }, ec);
            return ec == error::address_in_use;
        }

    } // namespace helper
} // namespace net
