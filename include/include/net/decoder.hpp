#pragma once
#include "net/types.hpp"
#include "net/streamer.hpp"
#include "Module/modules.h"
#include "udp_client.hpp"
#include "ipc_reader.hpp"

namespace net {
    using namespace navi;

    namespace udp {

        template <typename Src, typename Pkt>
        using reader = udp_reader<Src, Pkt>;

        namespace decoder {

            using VideoStream = DataIStreamer<VideoData, udp_reader>;
            using DepthStream = DataIStreamer<DepthData, udp_reader>;
            using ColorPCStream = DataIStreamer<ColorPCData, udp_reader>;
            using PosedPointStream = DataIStreamer<PosedPointData, udp_reader>;

            using VideoReader = VideoStream::net_type;
            using DepthReader = DepthStream::net_type;
            using ColorPCReader = ColorPCStream::net_type;
            using PosedPointReader = PosedPointStream::net_type;

        } // namespace decoder

        template <>
        bool decoder::VideoReader::decode(
            decoder::VideoReader::pack_type &packet, decoder::VideoReader::type &dst);
        template <>
        void decoder::VideoReader::exit();

        template <>
        bool decoder::DepthReader::decode(
            decoder::DepthReader::pack_type &packet, decoder::DepthReader::type &dst);

        template <>
        bool decoder::ColorPCReader::decode(
            decoder::ColorPCReader::pack_type &packet, decoder::ColorPCReader::type &dst);

        template <>
        bool decoder::PosedPointReader::decode(
            decoder::PosedPointReader::pack_type &packet, decoder::PosedPointReader::type &dst);

        //-------------------------------------------------------------------------

        // Fallback enter
        template <typename Src, typename Pkt>
        void udp_reader<Src, Pkt>::enter()
        {
        }

        // Fallback encode
        template <typename Src, typename Pkt>
        bool udp_reader<Src, Pkt>::decode(Pkt &pkt, Src &src)
        {
            throw std::runtime_error("The `encode` method not implemented.");
        }

        // Fallback exit
        template <typename Src, typename Pkt>
        void udp_reader<Src, Pkt>::exit()
        {
        }

    } // namespace udp

    namespace ipc {

        template <typename Src, typename Pkt>
        using reader = ipc_reader<Src, Pkt>;

        namespace decoder {

            using VideoStream = DataIStreamer<VideoData, ipc_reader>;
            using DepthStream = DataIStreamer<DepthData, ipc_reader>;
            using ColorPCStream = DataIStreamer<ColorPCData, ipc_reader>;
            using PosedPointStream = DataIStreamer<PosedPointData, ipc_reader>;

            using VideoReader = VideoStream::net_type;
            using DepthReader = DepthStream::net_type;
            using ColorPCReader = ColorPCStream::net_type;
            using PosedPointReader = PosedPointStream::net_type;

        } // namespace decoder

        template <>
        bool decoder::VideoReader::decode(
            decoder::VideoReader::pack_type &packet, decoder::VideoReader::type &dst);

        template <>
        bool decoder::DepthReader::decode(
            decoder::DepthReader::pack_type &packet, decoder::DepthReader::type &dst);

        template <>
        bool decoder::ColorPCReader::decode(
            decoder::ColorPCReader::pack_type &packet, decoder::ColorPCReader::type &dst);

        template <>
        bool decoder::PosedPointReader::decode(
            decoder::PosedPointReader::pack_type &packet, decoder::PosedPointReader::type &dst);

        //-------------------------------------------------------------------------

        // Fallback encode
        template <typename Src, typename Pkt>
        bool ipc_reader<Src, Pkt>::decode(Pkt &pkt, Src &src)
        {
            throw std::runtime_error("The `encode` method not implemented.");
        }
    } // namespace ipc

} // namespace net
