#include "net/helper.hpp"
#include "net/decoder.hpp"
#include "net/serialize.hpp"

#include <opencv2/opencv.hpp>
#if defined(CODEC_CUDA)
#include "ftl/nvidia_decoder.hpp"
#include "ftl/nvidia_encoder.hpp"
#else
#include "ftl/ffmpeg_decoder.hpp"
#include "ftl/ffmpeg_encoder.hpp"
#endif

#include "rvl.h"

namespace net { namespace udp {

    using namespace decoder;
    using pdecoder = std::shared_ptr<ftl::codecs::FFmpegDecoder>;

    static std::map<std::thread::id, pdecoder> decoders;

    pdecoder GetDecoder(std::thread::id tid)
    {
        if (decoders.find(tid) == decoders.end()) {
            // 创建编码器
            auto encoder = std::make_shared<pdecoder::element_type>();
            decoders.emplace(tid, encoder);
        }
        return decoders.at(tid);
    }

    void RemoveDecoder(std::thread::id tid)
    {
        auto it = decoders.find(tid);
        if (it != decoders.end()) {
            decoders.erase(it);
        }
    }

    template <>
    bool VideoReader::decode(VideoReader::pack_type &packet, VideoReader::type &dst)
    {
        auto &decoder = *GetDecoder(std::this_thread::get_id());

        ftl::codecs::Packet pkt;
        // 反序列化
        boost::iostreams::array_source source(packet.data.data(), packet.data.size());
        boost::iostreams::stream<boost::iostreams::array_source> is(source);
        boost::archive::binary_iarchive ia(is);
        ia >> pkt;
        dst.time = packet.time;
        return decoder.decode(pkt, dst.data) && !dst.data.empty();
    }

    template <>
    void VideoReader::exit()
    {
        RemoveDecoder(std::this_thread::get_id());
    }

    template <>
    bool decoder::DepthReader::decode(
        decoder::DepthReader::pack_type &packet, decoder::DepthReader::type &dst)
    {
        if (packet.dims.size() < 2) {
            return false;
        }
        double max;
        // 反序列化
        cv::Mat d;
        boost::iostreams::array_source source(packet.data.data(), packet.data.size());
        boost::iostreams::stream<boost::iostreams::array_source> is(source);
        boost::archive::binary_iarchive ia(is);
        ia >> d;
        ia >> max;
        if (d.rows == packet.dims[0] && d.cols == packet.dims[1]) {
            d.convertTo(dst.data, CV_32FC1, max / 65536.0);
            dst.time = packet.time;
            return true;
        }
        return false;
    }

    template <>
    bool ColorPCReader::decode(ColorPCReader::pack_type &packet, ColorPCReader::type &dst)
    {
        // 反序列化
        cv::Mat d;
        double min1, max1, min2, max2, min3, max3;
        boost::iostreams::array_source source(packet.data.data(), packet.data.size());
        boost::iostreams::stream<boost::iostreams::array_source> is(source);
        boost::archive::binary_iarchive ia(is);
        ia >> d >> min1 >> max1 >> min2 >> max2 >> min3 >> max3;

        if (d.rows == packet.dims[0] && d.cols == packet.dims[1]) {
            dst.time = packet.time;
            // 转换
            d.convertTo(dst.data, CV_32FC1, 1.0 / 255.0);

            {
                cv::Mat pcd = dst.data(cv::Rect(0, 0, 1, d.rows));
                cv::minMaxIdx(pcd, &min1, &max1);
                pcd = pcd * (max1 - min1) + min1;
            }
            {
                cv::Mat pcd = dst.data(cv::Rect(1, 0, 1, d.rows));
                cv::minMaxIdx(pcd, &min2, &max2);
                pcd = pcd * (max2 - min2) + min2;
            }
            {
                cv::Mat pcd = dst.data(cv::Rect(2, 0, 1, d.rows));
                cv::minMaxIdx(pcd, &min3, &max3);
                pcd = pcd * (max3 - min3) + min3;
            }

            return true;
        }
        return false;
    }

    bool PosedPointReader::decode(PosedPointReader::pack_type &packet, PosedPointReader::type &dst)
    {
        // 反序列化
        cv::Mat d;
        boost::iostreams::array_source source(packet.data.data(), packet.data.size());
        boost::iostreams::stream<boost::iostreams::array_source> is(source);
        boost::archive::binary_iarchive ia(is);
        ia >> d;
        if (d.rows == packet.dims[0] && d.cols == packet.dims[1]) {
            dst.time = packet.time;
            dst.data = d;
            return true;
        }
        return false;
    }

}} // namespace net::udp