#pragma once
#include <regex>
#include <boost/asio.hpp>

namespace net {

    struct _input_net_type {
    };

    struct _output_net_type {
    };

    namespace helper {

        static bool parse_ip(const std::string &source, std::string &addr, int &port)
        {
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
