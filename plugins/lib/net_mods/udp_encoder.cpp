#include "net/helper.hpp"
#include "net/encoder.hpp"
#include "net/serialize.hpp"

#include <opencv2/opencv.hpp>
// Encoder/Decoder
#include "ftl/packet.hpp"
#if defined(CODEC_CUDA)
#include <opencv2/cudaimgproc.hpp>
#include "ftl/nvidia_decoder.hpp"
#include "ftl/nvidia_encoder.hpp"
#else
#include "ftl/ffmpeg_decoder.hpp"
#include "ftl/ffmpeg_encoder.hpp"
#endif

#include "rvl.h"

namespace net { namespace udp {

    using namespace encoder;
    using pencoder = std::shared_ptr<ftl::codecs::FFmpegEncoder>;

    static std::map<std::thread::id, pencoder> encoders;

    pencoder GetEncoder(std::thread::id tid)
    {
        if (encoders.find(tid) == encoders.end()) {
            // 创建编码器
            ftl::codecs::EncoderParameters param;
            param.gop_size = 10;
            param.max_b_frames = 0;
            param.pix_fmt = AV_PIX_FMT_YUV420P;
            param.framerate = AVRational{ 30, 1 };
            param.tune = "zerolatency";
            param.preset = "veryfast";
            param.bitrate_max = 40.0; /* Mbps */
            auto encoder = std::make_shared<pencoder::element_type>(param);
            encoders.emplace(tid, encoder);
        }
        return encoders.at(tid);
    }

    void RemoveEncoder(std::thread::id tid)
    {
        auto it = encoders.find(tid);
        if (it != encoders.end()) {
            encoders.erase(it);
        }
    }

    template <>
    bool VideoSender::encode(const VideoSender::type &src, VideoSender::pack_type &packet)
    {
        if (src.data.empty()) {
            return false;
        }
        packet.time = src.time;

        auto encoder = GetEncoder(std::this_thread::get_id());

        // 编码
        ftl::codecs::Packet pkt;
        pkt.codec = ftl::codecs::codec_t::H264;
        pkt.bitrate = 32;
        if (!encoder->encode(src.data, pkt)) {
            return false;
        }

        packet.dims = { src.data.cols, src.data.rows, src.data.channels() };

        // 序列化
        auto i = boost::iostreams::back_inserter(packet.data);
        boost::iostreams::stream<decltype(i)> os(i);
        boost::archive::binary_oarchive oa(os);
        oa << pkt;
        os.flush();
        return true;
    }

    template <>
    void VideoSender::exit()
    {
        RemoveEncoder(std::this_thread::get_id());
    }

    template <>
    bool DepthSender::encode(const DepthSender::type &src, DepthSender::pack_type &packet)
    {
        if (src.data.empty()) {
            return false;
        }
        packet.time = src.time;

        double max, min;
        cv::minMaxIdx(src.data, &min, &max);
        max = std::max(max, 1.0);

        cv::Mat dst;
        src.data.convertTo(dst, CV_16UC1, 65536.0 / max);
        packet.dims = { (int)dst.rows, (int)dst.cols, dst.channels() };

        //// 压缩
        // auto sz = dst.rows * dst.cols;
        // auto pack = rvl::compress(reinterpret_cast<short *>(dst.ptr()), sz);
        // packet.dims = { (int)dst.rows, (int)dst.cols };
        // packet.data.insert(
        //    packet.data.begin(), (char *)pack.data(), (char *)(pack.data() + pack.size()));

        // 序列化
        auto i = boost::iostreams::back_inserter(packet.data);
        boost::iostreams::stream<decltype(i)> os(i);
        boost::archive::binary_oarchive oa(os);
        oa << dst;
        oa << max;
        os.flush();
        return true;
    }

    template <>
    bool ColorPCSender::encode(const ColorPCSender::type &src, ColorPCSender::pack_type &packet)
    {
        if (src.data.empty()) {
            return false;
        }
        packet.time = src.time;
        packet.dims = { (int)src.data.rows, (int)src.data.cols }; // row * 6

        double min1, max1;
        {
            cv::Mat pcd = src.data(cv::Rect(0, 0, 1, src.data.rows));
            cv::minMaxIdx(pcd, &min1, &max1);
            pcd = (pcd - min1) / (max1 - min1);
        }
        double min2, max2;
        {
            cv::Mat pcd = src.data(cv::Rect(1, 0, 1, src.data.rows));
            cv::minMaxIdx(pcd, &min2, &max2);
            pcd = (pcd - min2) / (max2 - min2);
        }
        double min3, max3;
        {
            cv::Mat pcd = src.data(cv::Rect(2, 0, 1, src.data.rows));
            cv::minMaxIdx(pcd, &min3, &max3);
            pcd = (pcd - min3) / (max3 - min3);
        }

        cv::Mat d;
        src.data.convertTo(d, CV_8UC1, 255.0);

        // 序列化
        auto i = boost::iostreams::back_inserter(packet.data);
        boost::iostreams::stream<decltype(i)> os(i);
        boost::archive::binary_oarchive oa(os);
        oa << d;
        oa << min1 << max1;
        oa << min2 << max2;
        oa << min3 << max3;
        os.flush();
        return true;
    }

    template <>
    bool PosedPointSender::encode(
        const PosedPointSender::type &src, PosedPointSender::pack_type &packet)
    {
        if (src.data.empty()) {
            return false;
        }
        packet.time = src.time;
        packet.dims = { (int)src.data.rows, (int)src.data.cols };

        // 序列化
        auto i = boost::iostreams::back_inserter(packet.data);
        boost::iostreams::stream<decltype(i)> os(i);
        boost::archive::binary_oarchive oa(os);
        oa << src.data;
        os.flush();
        return true;
    }
}} // namespace net::udp