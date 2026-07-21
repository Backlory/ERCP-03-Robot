#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>
#include <boost/signals2.hpp>
#include <fmt/format.h>

#include "ffmpeg_capture.hpp"
#include "videolist.h"
#include "utils.h"
#include "cvtask.hpp"
#include "cvui.hpp"

using namespace boost::placeholders;

#define ROBOT_ERROR(cond, x)                   \
    {                                          \
        if (cond) std::cerr << x << std::endl; \
    }
#define ROBOT_INFO(cond, x)                    \
    {                                          \
        if (cond) std::cout << x << std::endl; \
    }

int CvUI::reload_count = 0;

namespace ilsr {

    /******************** CvTask **********************/
    CvTask::CvTask() {
        // Register callback
        conn = CvVideo::get_instance().connect(boost::bind(&CvTask::on_update, this, _1));
    }

    CvTask::~CvTask() {
        close();
        // Unregister callback
        conn.disconnect();
    }

    void CvTask::run() {
        if (!is_run()) {
            m_thread.swap(std::make_shared<boost::thread>(boost::bind(&CvTask::runner, this)));
            on_start();
            std::cout << "Run task [" << get_name() << "]." << std::endl;
        }
    }

    void CvTask::close() {
        if (m_thread) {
            m_thread->interrupt();
            m_thread->join();
            on_close();
            std::cout << "Close task [" << get_name() << "]." << std::endl;
        }
        m_thread.reset();
    }

    bool CvTask::is_run() {
        return m_thread && m_thread->joinable();
    }

    int CvTask::runner() {

        cv::Mat frame;

        while (true) {
            // Wait for valid frame
            double ftime = -1;
            if (CvVideo::get_instance().get_frame(frame, 0.5, &ftime) && (!frame.empty())) {
                // Frame time is same.
                bool same_frame = std::abs(logged_ftime - ftime) < DBL_EPSILON;
                if (!same_frame) {
                    // Process frame
                    process(frame);
                }
                logged_ftime = ftime;
            }
            boost::this_thread::interruption_point();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }
    }

    /******************** CvVideo **********************/

    void CvVideo::run() {
        if (m_thread) { close(); }
#ifdef DEBUG
        std::cout << "Start video ..." << std::endl;
#endif  // DEBUG
        auto path = Settings::Get().Image.Path();
        if (!path || path->empty()) {
            m_thread.swap(std::make_shared<boost::thread>(boost::bind(&CvVideo::EndoScopySamplingWorker, this, 0, 15)));
        } else {
            bool pass = false;
            {
                try {
                    int v = std::atoi(path->c_str());
                    if (v >= 0 && v < 8) {
                        m_thread.swap(std::make_shared<boost::thread>(boost::bind(&CvVideo::EndoScopySamplingWorker, this, v, 15)));
                        pass = true;    
                    }
                }
                catch (...) {}
            }
            if(!pass)
                m_thread.swap(std::make_shared<boost::thread>(boost::bind(&CvVideo::EndoScopyVideoWorker, this, *path, 15)));
        }
    }

    void CvVideo::close() {
        if (m_thread) {
            m_thread->interrupt();
            m_thread->join();
#ifdef DEBUG
            std::cout << "Stop video." << std::endl;
#endif  // DEBUG
        }
        m_cap.reset();
        m_thread.reset();
    }

    bool CvVideo::is_run() {
        return m_thread && m_thread->joinable();
    }

    cv::Mat ilsr::get_roi(const cv::Mat &src, const cv::Rect &rect) {
        int l = std::max(rect.x, 1);
        int t = std::max(rect.y, 1);
        int r = std::min(rect.x + rect.width, src.cols - 1);
        int b = std::min(rect.y + rect.height, src.rows - 1);
        if (r - l <= 0 || b - t <= 0) { return cv::Mat(); }
        return src(cv::Rect(l, t, r - l, b - t));
    }

    bool CvVideo::get_frame(cv::Mat &frm, double overtime, double *ftime) {
        // Reader lock
        boost::shared_lock<boost::shared_mutex> lock(m_access);
        if (ilsr::Time::wall_time() - m_ftime < overtime) {
            m_frame.copyTo(frm);
            if (ftime) *ftime = m_ftime;
            return true;
        }
        return false;
    }

    double CvVideo::get_quality(double overtime) {
        // Reader lock
        boost::shared_lock<boost::shared_mutex> lock(m_access);
        if (ilsr::Time::wall_time() - m_ftime < overtime) { return m_quality; }
        return -1;
    }

    boost::signals2::connection CvVideo::connect(boost::function<void(cv::Mat &)> slot) {
        // Writer lock
        boost::upgrade_lock<boost::shared_mutex> lock(m_access);
        boost::upgrade_to_unique_lock<boost::shared_mutex> unique(lock);
        return on_frame_update.connect(slot);
    }

    CvVideo::CvVideo() {}

    CvVideo::~CvVideo() {
        close();
    }

}  // namespace ilsr

cv::Mat get_roi(const cv::Mat &src, const cv::Rect &rect) {
    int l = std::max(rect.x, 0);
    int t = std::max(rect.y, 0);
    int r = std::min(rect.x + rect.width, src.cols - 1);
    int b = std::min(rect.y + rect.height, src.rows - 1);
    if (r - l <= 0 || b - t <= 0) {
        return cv::Mat();
    }
    return src(cv::Rect(l, t, r - l + 1, b - t + 1));
}

namespace ilsr {

    int CvVideo::EndoScopySamplingWorker(int device, float fps) {

        bool tried = false;
        bool errored = false;
        cv::Rect ROI(0, 0, 0, 0);
        cv::Size SIZE = ROI.size();

        while (!boost::this_thread::interruption_requested()) {

            std::vector<DeviceInfo> devices;
            std::vector<std::vector<FormatInfo>> formats;
            std::vector<FormatInfo> possible;

            // 检索相机ID
            if (!GetAudioVideoInputDevices(devices, DeviceType::VideoInput)) {
                ROBOT_ERROR(!tried, "Cannot find camera device.")
                goto next;
            }
            if (device >= devices.size()) {
                device = 0;
            }
            if (!GetCameraFormats(formats, devices[device])) {
                ROBOT_ERROR(!tried, "Cannot read camera formats.")
                goto next;
            }
            // 获取不低于设置帧率的格式
            for (auto fmts : formats) {
                for (auto fmt : fmts) {
                    if (fmt.max.fps >= fps) {
                        possible.push_back(fmt);
                    }
                }
            }
            if (possible.size() <= 0) {
                ROBOT_ERROR(!tried, fmt::format("Cannot find camera format that fps >= {}.", fps))
                goto next;
            }

            {
                // 计算最佳格式
                FormatInfo best = *std::max_element(
                    possible.begin(), possible.end(), [](FormatInfo &a, FormatInfo &b) {
                        double scorea = a.max.width * a.max.height * a.max.fps;
                        double scoreb = b.max.width * b.max.height * b.max.fps;
                        return scorea < scoreb;
                    });

                ffmpeg_capture cap;
                cv::Mat src;
                cv::Mat m_rgb;
                double t;

                {
#ifdef VIDEOLIST_WINDOWS
                    std::string dev = ilsr::convert(devices[device].DeviceName);
#else
                    std::string dev = devices[id_dev].DeviceName;
#endif
                    std::string fmt_code, fmt;
                    if (!best.vcodec.empty()) {
                        fmt_code = "vcodec";
                        fmt = best.vcodec;
                    } else if (best.pixel_format != -1) {
                        fmt_code = "pixel_format";
                        fmt = av_get_pix_fmt_name((AVPixelFormat)best.pixel_format);
                    }
                    std::string siz = fmt::format("{}x{}", best.max.width, best.max.height);
                    std::string fps = fmt::format("{}", best.max.fps);

                    ROBOT_INFO(true,
                               fmt::format("Using camera device: {}"
                                           "\n    code/pix_fmt: {}"
                                           "\n    size: {}"
                                           "\n    fps: {}",
                                           dev, fmt, siz, fps));

                    AVDictionary *opt = NULL;
                    av_dict_set(&opt, fmt_code.c_str(), fmt.c_str(), 0);
                    av_dict_set(&opt, "video_size", siz.c_str(), 0);
                    // av_dict_set(&opt, "framerate", fps.c_str(), 0);
                    // cap.open("video=" + dev, inpfmt, &opt);
                    if (!cap.open(device, &opt)) {
                        ROBOT_ERROR(true, "Open camera failed: " << cap.get_error());
                    }
                    av_dict_free(&opt);
                }

                if (cap.is_opened()) {
                    tried = false;
                    errored = false;

                    try {
                        while (!boost::this_thread::interruption_requested()) {
                            if (!cap.read(src)) {
                                break;
                            }

                            if (ROI.width <= 0 || ROI.height <= 0) {
                                ROI.x = ROI.y = 0;
                                ROI.width = src.cols;
                                ROI.height = src.rows;
                            }
                            // 保证尺寸符合32整数倍数
                            if (ROI.width % 32 != 0) {
                                SIZE.width = int(ceil(ROI.width / 32.0)) * 32;
                            }
                            if (ROI.height % 32 != 0) {
                                SIZE.height = int(ceil(ROI.height / 32.0)) * 32;
                            }

                            auto frame = get_roi(src, ROI).clone();
                            if (frame.cols != SIZE.width || frame.rows != SIZE.height) {
                                ffmpeg_capture::resize(frame, frame, SIZE, AV_PIX_FMT_BGR24, SWS_BILINEAR);
                            }

                            // 计算图像清晰度
                            cv::Mat gray;
                            cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
                            cv::Mat sobel;
                            cv::Sobel(gray, sobel, CV_16U, 1, 1);
                            double meanValue2 = 0.0;
                            double img_quality = cv::mean(sobel)[0];
                            // 缓存图像
                            {
                                // Writer lock
                                boost::upgrade_lock<boost::shared_mutex> lock(m_access);
                                boost::upgrade_to_unique_lock<boost::shared_mutex> unique(lock);
                                frame.copyTo(m_frame);
                                m_quality = img_quality;
                                m_ftime = ilsr::Time::wall_time();
                            }

                            {
                                // Reader lock
                                boost::shared_lock<boost::shared_mutex> lock(m_access);
                                on_frame_update(frame);
                            }

                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            boost::this_thread::interruption_point();
                        }
                    } catch (std::exception &e) {
                        ROBOT_ERROR(true, "Camera error: " << e.what())
                    }
                }
            }

        next:
            ROBOT_INFO(!tried, "Camera is not opened, wait for opening ...")
            std::this_thread::sleep_for(std::chrono::seconds(3));
            tried |= true;
            boost::this_thread::interruption_point();
        }
        return 0;
    }

    int CvVideo::EndoScopyVideoWorker(std::string device, float fps) {

        bool tried = false;
        bool errored = false;
        cv::Rect ROI(0, 0, 0, 0);
        cv::Size SIZE = ROI.size();
        int scale = 0;

        while (!boost::this_thread::interruption_requested()) {

            cv::VideoCapture cap(device);

            if (cap.isOpened()) {
                tried = false;
                errored = false;

                try {
                    cv::Mat src;
                    while (!boost::this_thread::interruption_requested()) {
                        auto t0 = std::chrono::high_resolution_clock::now();
                        if (!cap.read(src)) {
                            break;
                        }

                        // 对图片进行缩放
                        if (scale != 0) {
                            cv::Size scale_size(int(src.cols * scale), int(src.rows * scale));
                            ffmpeg_capture::resize(src, src, scale_size, AV_PIX_FMT_BGR24, SWS_BILINEAR);
                        }

                        if (ROI.width <= 0 || ROI.height <= 0) {
                            ROI.x = ROI.y = 0;
                            ROI.width = src.cols;
                            ROI.height = src.rows;
                            SIZE = ROI.size();
                        }
                        // 保证尺寸符合32整数倍数
                        if (ROI.width % 32 != 0) {
                            SIZE.width = int(ceil(ROI.width / 32.0)) * 32;
                        }
                        if (ROI.height % 32 != 0) {
                            SIZE.height = int(ceil(ROI.height / 32.0)) * 32;
                        }

                        auto frame = get_roi(src, ROI).clone();
                        if (frame.cols != SIZE.width || frame.rows != SIZE.height) {
                            ffmpeg_capture::resize(frame, frame, SIZE, AV_PIX_FMT_BGR24, SWS_BILINEAR);
                        }

                        // 计算图像清晰度
                        cv::Mat gray;
                        cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
                        cv::Mat sobel;
                        cv::Sobel(gray, sobel, CV_16U, 1, 1);
                        double meanValue2 = 0.0;
                        double img_quality = cv::mean(sobel)[0];
                        // 缓存图像
                        {
                            // Writer lock
                            boost::upgrade_lock<boost::shared_mutex> lock(m_access);
                            boost::upgrade_to_unique_lock<boost::shared_mutex> unique(lock);
                            frame.copyTo(m_frame);
                            m_quality = img_quality;
                            m_ftime = ilsr::Time::wall_time();
                        }

                        {
                            // Reader lock
                            boost::shared_lock<boost::shared_mutex> lock(m_access);
                            on_frame_update(frame);
                        }

                        auto t1 = std::chrono::high_resolution_clock::now();
                        double dt = std::chrono::duration<double>(t1 - t0).count();
                        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, int((1.0 / fps - dt)) * 1000)));
                        boost::this_thread::interruption_point();
                    }
                } catch (std::exception &e) {
                    ROBOT_ERROR(true, "Camera error: " << e.what())
                }
            }

        next:
            ROBOT_INFO(!tried, "Camera is not opened, wait for opening ...")
            std::this_thread::sleep_for(std::chrono::seconds(3));
            tried |= true;
            boost::this_thread::interruption_point();
        }
        return 0;
    }

    int CvVideo::runner() {
        return 0;
    }
}  // namespace ilsr
