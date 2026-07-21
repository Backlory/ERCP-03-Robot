#pragma once
#include "cvtask.hpp"
#include "utils.h"

class RobotAct;

class CvUI : public ilsr::CvTask {

public:
    static CvUI &get_instance() {
        static CvUI _ui;
        return _ui;
    }

protected:
    CvUI();
    CvUI(const CvUI &) = delete;
    ~CvUI();

protected:
    std::string get_name() {
        return "OpenCV-UI";
    };

    void make_canvas();

    void process(cv::Mat &frame);

    void on_update(cv::Mat &src);

    void on_start();

    void on_close();

private:
    cv::Mat m_canvas;
    std::string m_name;

    static std::shared_ptr<ilsr::Logger> m_log;

    static int reload_count;

    // 加载资源
    std::array<cv::Mat, 8> image_digest;
    cv::Mat img_warning;
    cv::Mat logo_sia;
    cv::Mat logo_301;

public:
    // 常量
    static const std::array<std::string, 4> str_angle;
};
