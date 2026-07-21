/**
 * @file ffmpeg_capture.hpp
 * @copyright Copyright (c) 2022 SIA-ILSR, MIT License
 * @author YTom
 */

#ifndef _FFMPEG_CAPTURE_HPP_
#define _FFMPEG_CAPTURE_HPP_

#include <iostream>
#include <vector>
#include <locale>
#include <codecvt>
#include <opencv2/core.hpp>
#include "ffmpeg.h"
#include "videolist.h"

static cv::Mat avframeToCvmat(const AVFrame *frame)
{
    int width = frame->width;
    int height = frame->height;
    cv::Mat image(height, width, CV_8UC3);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();
    SwsContext *conversion = sws_getContext(width, height, (AVPixelFormat)frame->format, width,
        height, AVPixelFormat::AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
    sws_freeContext(conversion);
    return image;
}

class ffmpeg_capture {

public:
    struct Size_t {
        int width = 0;
        int height = 0;
    };

    ffmpeg_capture() { av_log_set_level(AV_LOG_FATAL); }

    ffmpeg_capture(const std::string &infile)
        : ffmpeg_capture()
    {
        open(infile);
    }

    ffmpeg_capture(const int camera_id)
        : ffmpeg_capture()
    {
        open(camera_id);
    }

    ~ffmpeg_capture() { close(); }

public:
    static bool resize(const cv::Mat &src, cv::Mat &dst, cv::Size size,
        AVPixelFormat ifmt = AV_PIX_FMT_BGR24, int method = SWS_BICUBIC)
    {

        // TODO: 肯需要一些检查
        SwsContext *ctx = sws_getContext(src.cols, src.rows, ifmt, size.width, size.height,
            AV_PIX_FMT_BGR24, method, NULL, NULL, NULL);
        if (!ctx) {
            return false;
        }

        cv::Mat _dst(size.height, size.width, src.type());

        int stride = src.step1();
        int stride2 = _dst.step1();
        sws_scale(ctx, &src.data, &stride, 0, src.rows, &_dst.data, &stride2);
        _dst.copyTo(dst);
        sws_freeContext(ctx);
        return true;
    }

public:
#ifdef VIDEOLIST_WINDOWS
    bool open(const int camera_id, AVDictionary **options = NULL)
    {
        avdevice_register_all();
        AVInputFormat *fmt = av_find_input_format("dshow");
        if (!fmt) {
            throw std::runtime_error("Not found dshow");
        }

        std::vector<DeviceInfo> vectorDevices;
        GetAudioVideoInputDevices(vectorDevices, DeviceType::VideoInput);
        if (vectorDevices.size() > camera_id) {
            std::string camName = wstring2string(vectorDevices[camera_id].DeviceName);
            std::string temp = "video=" + camName;
            return open(temp, fmt, options);
        } else {
            m_error = "invalid camera id!";
        }
        return false;
    }
#else
    // TODO:
    bool open(const int camera_id, AVDictionary **options = NULL)
    {
        avdevice_register_all();
        // AVInputFormat *fmt = av_find_input_format("dshow");
        // if (!fmt) {
        //     throw std::runtime_error("Not found dshow");
        // }

        // std::vector<DeviceInfo> vectorDevices;
        // GetAudioVideoInputDevices(vectorDevices, DeviceType::VideoInput);
        // if (vectorDevices.size() > camera_id) {
        //     std::string camName = wstring2string(vectorDevices[camera_id].DeviceName);
        //     std::string temp = "video=" + camName;
        //     return open(temp, fmt, options);
        // } else {
        //     m_error = "invalid camera id!";
        // }
        return false;
    }
#endif

    bool open(const std::string &infile, AVInputFormat *fmt = NULL, AVDictionary **options = NULL)
    {

        if (infile.empty()) {
            return false;
        }
        if (inctx)
            return true;

        int ret;
        bool succ = true;

        // open input file context
        inctx = NULL;
        ret = avformat_open_input(&inctx, infile.c_str(), fmt, options);
        if (ret < 0) {
            char buff[512];
            av_strerror(ret, buff, 512);
            m_error = "fail to dshow_open_input(`" + infile + "`). " + buff;
            succ = false;
        }

        // retrive input stream information
        if (succ) {
            ret = avformat_find_stream_info(inctx, NULL);
            if (ret < 0) {
                char buff[512];
                av_strerror(ret, buff, 512);
                m_error = "fail to avformat_find_stream_info";
                succ = false;
            }
        }

        // find primary video stream
        AVCodec *vcodec = NULL;
        if (succ) {
            ret = av_find_best_stream(inctx, AVMEDIA_TYPE_VIDEO, -1, -1, &vcodec, 0);
            if (ret < 0) {
                char buff[512];
                av_strerror(ret, buff, 512);
                m_error = std::string("fail to av_find_best_stream. ") + buff;
                succ = false;
            }
        }

        // open video decoder context
        AVStream *vstrm = NULL;
        if (succ) {
            vstrm_idx = ret;
            vstrm = inctx->streams[vstrm_idx];

            // Find decoder for stream
            auto codec = avcodec_find_decoder(vstrm->codecpar->codec_id);
            if (!codec) {
                char buff[512];
                av_strerror(ret, buff, 512);
                m_error = std::string("fail to avcodec_find_decoder. ") + buff;
                succ = false;
            }
            incodec = avcodec_alloc_context3(codec);
            if (!incodec) {
                char buff[512];
                av_strerror(ret, buff, 512);
                m_error = std::string("fail to avcodec_alloc_context3. ") + buff;
                succ = false;
            }
            ret = avcodec_parameters_to_context(incodec, vstrm->codecpar);
            if (ret < 0) {
                char buff[512];
                av_strerror(ret, buff, 512);
                m_error = std::string("fail to avcodec_parameters_to_context. ") + buff;
                succ = false;
            }
            ret = avcodec_open2(incodec, vcodec, NULL);
            if (ret < 0) {
                char buff[512];
                av_strerror(ret, buff, 512);
                m_error = std::string("fail to avcodec_open2. ") + buff;
                succ = false;
            }
        }

        if (!succ) {
            close();
        }

        return succ;
    }

    AVFrame *read(AVFrame *frame = NULL)
    {

        if (!inctx)
            return NULL;

        bool res = true;

        // decoding loop
        AVPacket pkt;
        // allocate frame buffer for output
        if (frame == NULL) {
            frame = av_frame_alloc();
        }

        // read packet from input file
        int rsize = av_read_frame(inctx, &pkt);
        if (rsize < 0 || rsize == AVERROR_EOF || pkt.stream_index != vstrm_idx) {
            res = false;
        } else {
            // decode video frame
            // [Desperated] avcodec_decode_video2(vstrm->codec, decframe, &got_pic, &pkt);
            if (avcodec_send_packet(incodec, &pkt) < 0) {
                res = false;
            } else {
                res = (avcodec_receive_frame(incodec, frame) == 0);
            }
        }
        av_packet_unref(&pkt);
        return res ? frame : NULL;
    }

    bool read(cv::Mat &src)
    {

        if (!inctx)
            return false;

        bool res = true;

        // decoding loop
        AVPacket pkt;
        AVFrame *frame = av_frame_alloc();

        // read packet from input file
        int rsize = av_read_frame(inctx, &pkt);
        if (rsize < 0 || rsize == AVERROR_EOF || pkt.stream_index != vstrm_idx) {
            res = false;
        } else {
            if (avcodec_send_packet(incodec, &pkt) < 0) {
                res = false;
            } else {
                if (avcodec_receive_frame(incodec, frame) == 0) {
                    extern cv::Mat avframeToCvmat(const AVFrame *frame);
                    auto frm = avframeToCvmat(frame);
                    frm.copyTo(src);
                    res = true;
                }
            }
        }
        av_packet_unref(&pkt);
        av_frame_free(&frame);
        return res;
    }

    void close()
    {
        if (incodec) {
            avcodec_close(incodec);
            incodec = NULL;
        }
        if (inctx) {
            avformat_close_input(&inctx);
            inctx = NULL;
        }
        vstrm_idx = -1;
    }

    bool is_opened() { return inctx != NULL; }

    std::string get_error() { return m_error; }

    std::string wstring2string(std::wstring wstr)
    {
        std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
        return converter.to_bytes(wstr);
    }

private:
    std::string m_error = "";
    AVFormatContext *inctx = NULL;
    AVCodecContext *incodec = NULL;
    int vstrm_idx = -1;
};

#endif // _FFMPEG_CAPTURE_HPP_
