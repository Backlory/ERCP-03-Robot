#include "net/helper.hpp"
#include "net/decoder.hpp"
#include "net/serialize.hpp"

#define DESERIALIZE_CVMAT                                                                          \
    if (packet.dims.size() >= 4) {                                                                 \
        size_t sz = packet.dims[0] * packet.dims[1] * packet.dims[3];                              \
        if (sz == packet.data.size()) {                                                            \
            cv::Mat d(packet.dims[0], packet.dims[1], packet.dims[2], packet.data.data());         \
            dst.data = d.clone();                                                                  \
            return true;                                                                           \
        }                                                                                          \
    }                                                                                              \
    return false;

namespace net { namespace ipc {

    using namespace decoder;

    template <>
    bool VideoReader::decode(VideoReader::pack_type &packet, VideoReader::type &dst)
    {
        DESERIALIZE_CVMAT
    }

    template <>
    bool decoder::DepthReader::decode(
        decoder::DepthReader::pack_type &packet, decoder::DepthReader::type &dst)
    {
        DESERIALIZE_CVMAT
    }

    template <>
    bool ColorPCReader::decode(ColorPCReader::pack_type &packet, ColorPCReader::type &dst)
    {
        DESERIALIZE_CVMAT
    }

    template <>
    bool PosedPointReader::decode(PosedPointReader::pack_type &packet, PosedPointReader::type &dst)
    {
        DESERIALIZE_CVMAT
    }

}} // namespace net::ipc