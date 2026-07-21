#pragma once
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <beauty/beauty.hpp>
#include <Poco/Net/SocketAddress.h>
#include "robot_config.h"
#include "packet.hpp"
#include "helper.hpp"
#include "utils.h"

namespace net { namespace udp {

    using namespace Poco::Net;
    using sequence_t = SplitPacketQueue_t;

    template <typename Src, typename Pkt>
    struct udp_reader {
        using type = Src;
        using pack_type = Pkt;
        using direction = _input_net_type;

        udp_reader(const std::string &address, int verbose);
        ~udp_reader();

        std::string get_name() const { return name; }
        std::string get_address() const { return address.toString(); }
        bool is_running() const { return m_stream != nullptr; }

        void enter();
        bool decode(Pkt &pkt, Src &src);
        void exit();

        const int verbose;
        ilsr::mutex_data<Src> frame;

    private:
        const std::string name;
        const SocketAddress address;

        sequence_t sequence;
        std::atomic<size_t> counter;

        std::shared_ptr<boost::thread> m_thread;
        std::shared_ptr<beauty::udp_client> m_stream;
        std::shared_ptr<beauty::udp_callback> m_handler;

        void DecodeWorker();
    };

    template <typename Src, typename Pkt>
    udp_reader<Src, Pkt>::udp_reader(const std::string &address, int verbose)
        : name("udp/" + address)
        , address(address)
        , verbose(verbose)
    {
        // 创建采样视频流UDP接收回调函数
        m_handler = std::make_shared<beauty::udp_callback>();
        m_handler->on_read = [&](auto &, boost::asio::streambuf &buf, size_t size) {
            try {
                auto ptr = (char *)buf.data().data();
                std::vector<char> d(ptr, ptr + buf.data().size());
                if (!sequence.solve(d)) {
                    ROBOT_INFO(verbose > 1, "Bad saving new packet.")
                }
            } catch (std::exception &e) {
                ROBOT_INFO(verbose > 1, "[Error] " << e.what())
            }
            return true;
        };
        m_handler->on_read_failed = [&](auto &, auto) { //
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            return true; // return `true` to restart read
        };

        // 创建启动UDP流接收
        int port;
        std::string addr;
        if (!net::helper::parse_ip(address, addr, port)) {
            throw std::runtime_error("Invalid IP address.");
        }
        if (port <= 0) {
            throw std::runtime_error("Invalid port value.");
        }
        m_stream = std::make_shared<beauty::udp_client>(fmt::format("UDP/{}", name));
        m_stream->receive(port, *m_handler, true, verbose);
        // 创建解码码线程
        m_thread = std::make_shared<boost::thread>(boost::bind(&udp_reader::DecodeWorker, this));
        // ROBOT_INFO(verbose > 0, fmt::format("UDP stream reader start on {}", address.port()))
    }

    template <typename Src, typename Pkt>
    udp_reader<Src, Pkt>::~udp_reader()
    {
        if (m_thread) {
            m_thread->interrupt();
            m_thread->join();
        }
        m_stream.reset();
    }

    template <typename Src, typename Pkt>
    void udp_reader<Src, Pkt>::DecodeWorker()
    {
        ROBOT_THREADNAME(fmt::format("Dec/{}", name).c_str())

        enter();
        counter = 0;

        while (!boost::this_thread::interruption_requested()) {
            try {
                auto packs = sequence.reform<Pkt>();
                if (packs.size() > 0) {
                    Src src;
                    if (decode(packs.back()->packet, src)) {
                        // 更新缓存
                        frame.Set(src);
                        counter++;
                    }
                }
            } catch (const std::exception &e) {
                ROBOT_ERROR(verbose > 0, "[Error] " << e.what())
            }
            boost::this_thread::interruption_point();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        exit();
    }

}} // namespace net::udp