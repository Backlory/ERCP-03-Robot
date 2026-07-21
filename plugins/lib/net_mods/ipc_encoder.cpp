#include "net/helper.hpp"
#include "net/encoder.hpp"
#include "net/serialize.hpp"

#define SERIALIZE_CVMAT                                                                            \
    if (src.data.empty()) {                                                                        \
        return false;                                                                              \
    }                                                                                              \
    packet.dims = { src.data.rows, src.data.cols, src.data.type(), (int64_t)src.data.elemSize(),   \
        src.data.channels() };                                                                     \
    size_t sz = src.data.rows * src.data.cols * src.data.elemSize();                               \
    packet.data = std::vector<char>((char *)src.data.ptr(), (char *)src.data.ptr() + sz);          \
    return true;

// ------------------------------------------------------------------------

namespace net { namespace ipc {

    using namespace encoder;

    template <>
    bool VideoSender::encode(const VideoSender::type &src, VideoSender::pack_type &packet)
    {
        SERIALIZE_CVMAT
    }

    template <>
    bool DepthSender::encode(const DepthSender::type &src, DepthSender::pack_type &packet)
    {
        SERIALIZE_CVMAT
    }

    template <>
    bool ColorPCSender::encode(const ColorPCSender::type &src, ColorPCSender::pack_type &packet)
    {
        SERIALIZE_CVMAT
    }

    template <>
    bool PosedPointSender::encode(
        const PosedPointSender::type &src, PosedPointSender::pack_type &packet)
    {
        SERIALIZE_CVMAT
    }

}} // namespace net::ipc