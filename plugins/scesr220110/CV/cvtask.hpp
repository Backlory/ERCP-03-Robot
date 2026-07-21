#pragma once

#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>
#include <boost/signals2.hpp>
#include "Settings.hpp"

namespace pt = boost::posix_time;

class ffmpeg_capture;

namespace ilsr {

    typedef struct {
        double timestamp;
        cv::Point2d relpos;
    } TrackPos_t;

    typedef struct {
        int object;
        double score;
        cv::Rect2d region;
    } Target_t;

    typedef struct {
        double timestamp;
        double deltatime;
        std::vector<Target_t> targets;
    } TargetGroup_t;

    typedef struct {
        double timestamp;
        cv::Mat frame;
        std::vector<cv::Rect> rois;
    } BufferedFrame_t;

    cv::Mat get_roi(const cv::Mat &src, const cv::Rect &rect);

    class CvHandler {
    public:
        CvHandler() {
            m_Selector = std::make_shared<Selector_t>();
        }

        /**
         * @brief 选择区域记录
         * @note Source: opencv/modules/highgui/src/window_w32.cpp
         */
        typedef struct Selector_t {
            // basic parameters
            bool isDrawing = false;
            bool isPausing = false;
            cv::Rect box;
            cv::Mat image;
            cv::Mat output;
            cv::Point2f startPos;
            cv::Size imageSize;

            // parameters for drawing from the center
            bool drawFromCenter;

            // initializer list
            Selector_t()
                : isDrawing(false), drawFromCenter(true){};
        };

        std::shared_ptr<Selector_t> m_Selector;

        /**
         * @brief 更新和绘制选择区域
         *
         * @param img 输入退休
         * @param key 输入按键值
         * @param rect [Out]选择区域
         * @param showCrossair 是否显示十字线
         * @param fromCenter 是否从中心开始选择
         * @return true 确认选择区域（输入Enter）
         * @return false 未确认选择区域等其他情况
         * @note Source: opencv/modules/highgui/src/roiSelector.cpp
         */
        static bool UpdateSelect(Selector_t &selector, int key, bool showCrossair = true, bool fromCenter = false);

        /**
         * @brief 鼠标事件处理
         *
         * @param event
         * @param x
         * @param y
         * @param flags
         * @param param
         * @note Source: opencv/modules/highgui/src/window_w32.cpp
         */
        static void mouseHandler(int event, int x, int y, int flags, void *param);

        /**
         * @brief 空的鼠标事件处理，用于清除鼠标事件回调
         * @note Source: opencv/modules/highgui/src/window_w32.cpp
         */
        static void emptyMouseHandler(int, int, int, int, void *) {}

        /**
         * @brief Opencv源码的鼠标事件回调函数
         *
         * @param event
         * @param x
         * @param y
         * @note Source: opencv/modules/highgui/src/window_w32.cpp
         */
        static void opencv_mouse_callback(Selector_t &selector, int event, int x, int y, int);
    };

    class CvVideo {

    public:
        static CvVideo &get_instance() {
            static CvVideo _cv;
            return _cv;
        }

        void run();
        void close();
        bool is_run();

        /// <summary>
        /// Ger current frame.
        /// </summary>
        /// <param name="frm">Output the curremt frame.</param>
        /// <param name="overtime">Max delta time between current time and the frame time.</param>
        /// <param name="ftime">Output the frame time.</param>
        /// <returns></returns>
        bool get_frame(cv::Mat &frm, double overtime = 0.5, double *ftime = NULL);
        double get_quality(double overtime = 0.5);

        boost::signals2::connection connect(boost::function<void(cv::Mat &)> slot);

    protected:
        CvVideo();
        CvVideo(const CvVideo &) = delete;
        ~CvVideo();

    private:
        int runner();

        std::shared_ptr<boost::thread> m_thread = nullptr;
        std::shared_ptr<ffmpeg_capture> m_cap = nullptr;

        cv::Mat m_frame;
        double m_ftime = -1;
        double m_quality = -1;
        boost::shared_mutex m_access;

        int EndoScopySamplingWorker(int device, float fps);
        int EndoScopyVideoWorker(std::string device, float fps);

        /// <summary>
        /// OpenCV-UI显示更新事件
        /// </summary>
        boost::signals2::signal<void(cv::Mat &)> on_frame_update;
    };

    class CvTask {

    public:
        CvTask();
        ~CvTask();

        void run();
        void close();
        bool is_run();

    protected:
        /// <summary>
        /// 处理当前帧
        /// </summary>
        /// <param name="frame"></param>
        virtual void process(cv::Mat &frame) = 0;

        /// <summary>
        /// OpenCV-UI显示更新回调
        /// </summary>
        /// <param name="frame"></param>
        virtual void on_update(cv::Mat &frame) = 0;

        virtual void on_start() {}
        virtual void on_close() {}

        /// <summary>
        /// 当前任务的名称
        /// </summary>
        /// <returns></returns>
        virtual std::string get_name() = 0;

        double m_ftime = -1;
        boost::shared_mutex m_access;

    private:
        int runner();

        double logged_ftime = -1;
        boost::signals2::connection conn;
        std::shared_ptr<boost::thread> m_thread = nullptr;
    };

}  // namespace ilsr
