#pragma once
#include "net/types.hpp"
#include "net/streamer.hpp"
#include "Module/modules.h"
#include "udp_server.hpp"
#include "ipc_sender.hpp"

namespace net {
    using namespace navi;

    namespace udp {

        template <typename Src, typename Pkt>
        using sender = udp_sender<Src, Pkt>;

        namespace encoder {

            using VideoStream = DataOStreamer<VideoData, udp_sender>;
            using DepthStream = DataOStreamer<DepthData, udp_sender>;
            using ColorPCStream = DataOStreamer<ColorPCData, udp_sender>;
            using PosedPointStream = DataOStreamer<PosedPointData, udp_sender>;

            using VideoSender = VideoStream::net_type;
            using DepthSender = DepthStream::net_type;
            using ColorPCSender = ColorPCStream::net_type;
            using PosedPointSender = PosedPointStream::net_type;

        } // namespace encoder

        template <>
        bool encoder::VideoSender::encode(
            const encoder::VideoSender::type &src, encoder::VideoSender::pack_type &packet);
        template <>
        void encoder::VideoSender::exit();

        template <>
        bool encoder::DepthSender::encode(
            const encoder::DepthSender::type &src, encoder::DepthSender::pack_type &packet);

        template <>
        bool encoder::ColorPCSender::encode(
            const encoder::ColorPCSender::type &src, encoder::ColorPCSender::pack_type &packet);

        template <>
        bool encoder::PosedPointSender::encode(const encoder::PosedPointSender::type &src,
            encoder::PosedPointSender::pack_type &packet);

        //-------------------------------------------------------------------------

        // Fallback enter
        template <typename Src, typename Pkt>
        void udp_sender<Src, Pkt>::enter()
        {
        }

        // Fallback encode
        template <typename Src, typename Pkt>
        bool udp_sender<Src, Pkt>::encode(const Src &src, Pkt &pkt)
        {
            throw std::runtime_error("The `encode` method not implemented.");
        }

        // Fallback exit
        template <typename Src, typename Pkt>
        void udp_sender<Src, Pkt>::exit()
        {
        }

    } // namespace udp

    namespace ipc {

        template <typename Src, typename Pkt>
        using sender = ipc_sender<Src, Pkt>;

        namespace encoder {

            using VideoStream = DataOStreamer<VideoData, ipc_sender>;
            using DepthStream = DataOStreamer<DepthData, ipc_sender>;
            using ColorPCStream = DataOStreamer<ColorPCData, ipc_sender>;
            using PosedPointStream = DataOStreamer<PosedPointData, ipc_sender>;

            using VideoSender = VideoStream::net_type;
            using DepthSender = DepthStream::net_type;
            using ColorPCSender = ColorPCStream::net_type;
            using PosedPointSender = PosedPointStream::net_type;

        } // namespace encoder

        template <>
        bool encoder::VideoSender::encode(
            const encoder::VideoSender::type &src, encoder::VideoSender::pack_type &packet);

        template <>
        bool encoder::DepthSender::encode(
            const encoder::DepthSender::type &src, encoder::DepthSender::pack_type &packet);

        template <>
        bool encoder::ColorPCSender::encode(
            const encoder::ColorPCSender::type &src, encoder::ColorPCSender::pack_type &packet);

        template <>
        bool encoder::PosedPointSender::encode(const encoder::PosedPointSender::type &src,
            encoder::PosedPointSender::pack_type &packet);

        //-------------------------------------------------------------------------

        // Fallback encode
        template <typename Src, typename Pkt>
        bool ipc_sender<Src, Pkt>::encode(const Src &src, Pkt &pkt)
        {
            throw std::runtime_error("The `encode` method not implemented.");
        }
    } // namespace ipc

} // namespace net
