#pragma once
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <Poco/Net/DatagramSocket.h>
#include "helper.hpp"
#include "packet.hpp"
#include "robot_config.h"
#include "utils.h"

namespace net { namespace udp {

    using namespace Poco::Net;
    using sequence_t = SplitPacketQueue_t;

    template <typename Src, typename Pkt>
    class udp_sender {
    public:
        using type = Src;
        using pack_type = Pkt;
        using direction = _output_net_type;

        udp_sender(const std::string &address, int verbose);
        ~udp_sender();

        std::string get_name() const { return name; }
        std::string get_address() const
        {
            return fmt::format(
                "{}:{}", m_stream->address().host().toString(), m_stream->peerAddress().port());
        }
        bool is_running() const { return m_stream != nullptr; }

        void enter();
        bool encode(const Src &src, Pkt &pkt);
        void exit();

        const int verbose;
        ilsr::mutex_data<Src> frame;

    private:
        const std::string name;
        const std::string address;

        std::atomic<size_t> counter;

        std::shared_ptr<boost::thread> m_thread;
        std::shared_ptr<DatagramSocket> m_stream;

        void EncodeAndTransmitWorker();
    };

    template <typename Src, typename Pkt>
    udp_sender<Src, Pkt>::udp_sender(const std::string &address, int verbose)
        : name("udp/" + address)
        , address(address)
        , verbose(verbose)
    {
        int port;
        std::string addr;
        if (!net::helper::parse_ip(address, addr, port)) {
            throw std::runtime_error("Invalid IP address.");
        }
        if (port <= 0) {
            throw std::runtime_error("Invalid IP port.");
        }

        SocketAddress sa(addr, port);
        m_stream = std::make_shared<DatagramSocket>();
        m_stream->connect(sa);
        // ROBOT_INFO(verbose > 0, fmt::format("UDP stream server start on {}",
        // m_stream->address().toString()))
        m_thread = std::make_shared<boost::thread>(
            boost::bind(&udp_sender::EncodeAndTransmitWorker, this));
    }

    template <typename Src, typename Pkt>
    udp_sender<Src, Pkt>::~udp_sender()
    {
        if (m_thread) {
            m_thread->interrupt();
            m_thread->join();
        }
        m_stream.reset();
    }

    template <typename Src, typename Pkt>
    void udp_sender<Src, Pkt>::EncodeAndTransmitWorker()
    {
        ROBOT_THREADNAME(fmt::format("Enc/{}", name).c_str())

        enter();
        while (!boost::this_thread::interruption_requested()) {
            try {
                Src src;
                if (frame.TryMove(src, 0.2)) {
                    counter++;
                    Pkt pkt;
                    if (encode(src, pkt)) {
                        // 序列化结构体数据到数据流,分包发送
                        auto ps = sequence_t::pack(counter, pkt, 800);
                        for (auto &p : ps) {
                            if (m_stream) {
                                m_stream->sendBytes((void *)p.data(), p.size());
                            }
                        }
                    }
                }
            } catch (const std::exception &e) {
                ROBOT_ERROR(verbose > 0, "[Error] " << e.what());
            }
            boost::this_thread::interruption_point();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        exit();
    }

}} // namespace net::udp