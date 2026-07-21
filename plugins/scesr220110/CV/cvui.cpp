#include <fmt/format.h>
#include "CV/module/nanodet.hpp"
#include "CV/module/elm.hpp"
#include "CV/recog.hpp"
#include "Robot/VBot.hpp"
#include "cvui.hpp"
//#include "RobotAct.h"

#define BUFFER_SIZE 512
using namespace ilsr;

static std::string DoubleToString(double d)
{
    std::string str;
    std::stringstream ss;
    ss << d;
    ss >> str;
    return str;
}

const std::array<std::string, 4> CvUI::str_angle = { "Up:", "Down:", "Left:", "Right:" };

// 左侧面板区域
const cv::Rect A1(5, 5, 515, 805);
// 面板文字
const int TAG_L = 20;
const int TAG_T1 = 35;
const int TAG_T2 = 125;
const int TAG_T3 = 215;
const int TAG_T4 = 305;
// 操作指示图片位置
const cv::Rect OpeImg(260, 10, 240, 480);
// 弯曲指示
const int TAG_ITEM_L = 280;
const int TAG_VALUE_L = 380;
const int TAG_UNIT_L = 450;
const int TAG_H0 = 550;
// 弯曲显示区域
const int WS_Pr = 4; // Point radius
const int WS_Xc = 140; // 50 ~ 140 ~ 230 (original designed)
const int WS_Yc = 660; // 520 ~ 690 ~ 780 (original designed)
struct {
    double X = 170, x = 120, Y = 90, y = 80;
} WS;

// 内窥镜图像显示位置
const cv::Rect BFrame(530, 10, 900, 675);
//// 图像操作指示
// const int RTAG_L = 560;
// const int RTAG_H1 = 720;
// const int RTAG_H2 = 760;
// 力显示框的坐标
const int SP_X = 1000;
const int SP_Y = 840;
const int EP_X = 1420;
const int EP_Y = 880;

const int SP_X2 = 400;
const int SP_Y2 = 840;
const int EP_X2 = 820;
const int EP_Y2 = 880;

// LOGO显示
const int logo_w = 80;
const int logo_h = 80;

CvUI::CvUI()
{
    using namespace cv;
    using namespace std;

    static const std::string paths[] = {
        "data/scesr/res/Oringin.png",
        "data/scesr/res/Throughing Mouth.png",
        "data/scesr/res/Forwarding Esophagus.png",
        "data/scesr/res/Throughing Cardia.png",
        "data/scesr/res/Observing Gastric_body.png",
        "data/scesr/res/Observing Antrum.png",
        "data/scesr/res/Throughing Pylorus.png",
        "data/scesr/res/Observing Fundus.png",
    };

    // 加载资源
    for (int i = 0; i < 8; ++i) {
        image_digest[i] = imread(paths[i], 1);
        resize(image_digest[i], image_digest[i], OpeImg.size());
    }

    img_warning = imread("data/scesr/res/Warning.png", 1);
    logo_sia = imread("data/scesr/res/SIA.png", 1);
    logo_301 = imread("data/scesr/res/301.png", 1);
    resize(img_warning, img_warning, Size(logo_w, logo_h));
    resize(logo_301, logo_301, Size(logo_w, logo_h));
    resize(logo_sia, logo_sia, Size(logo_w, logo_h));

    // Here X,Y for left and upper , x, y for right and bottom.
    // Note: Y axis is inverse for the joystick, so min_position for max upper angle and small row
    // index.
    WS.Y = 130;
    WS.y = 200;
    WS.X = 100;
    WS.x = 100;

    make_canvas();

    m_name = fmt::format("Display-{}", reload_count);
}

CvUI::~CvUI() { close(); }

void CvUI::make_canvas()
{

    using namespace cv;

    m_canvas = Mat(900, 1440, CV_8UC3, Scalar(0, 0, 0));

    //***********绘制各部分窗口布局*************//
    rectangle(m_canvas, A1.tl(), A1.br(), Scalar(200, 200, 200), 1);
    line(m_canvas, cv::Point(5, A1.br().y), cv::Point(1440, A1.br().y), Scalar(200, 200, 200), 1);

    //***********各个部分标签*************//
    putText(m_canvas, "Recognized activity:", cv::Point(TAG_L, TAG_T1), cv::FONT_HERSHEY_COMPLEX,
        0.6, cv::Scalar(255, 255, 255), 0.4, 8);
    putText(m_canvas, "Operation detection:", cv::Point(TAG_L, TAG_T2), cv::FONT_HERSHEY_COMPLEX,
        0.6, cv::Scalar(255, 255, 255), 0.4, 8);
    putText(m_canvas, "Intervention State:", cv::Point(TAG_L, TAG_T3), cv::FONT_HERSHEY_COMPLEX,
        0.6, cv::Scalar(255, 255, 255), 0.4, 8);
    putText(m_canvas, "Image quality:", cv::Point(TAG_L, TAG_T4), cv::FONT_HERSHEY_COMPLEX, 0.6,
        Scalar(255, 255, 255), 0.4);

    for (int i = 0; i < 4; i++) {
        putText(m_canvas, str_angle[i], cv::Point(TAG_ITEM_L, TAG_H0 + (i * 70)),
            cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 255, 255), 0.4, 8);
        putText(m_canvas, "deg", cv::Point(TAG_UNIT_L, TAG_H0 + (i * 70)), cv::FONT_HERSHEY_COMPLEX,
            0.8, cv::Scalar(200, 200, 200), 0.4, 8);
    }

    putText(m_canvas, "Force:", cv::Point(900, 875), cv::FONT_HERSHEY_COMPLEX, 0.6,
        Scalar(255, 255, 255), 0.4);
    putText(m_canvas, "Length:", cv::Point(310, 875), cv::FONT_HERSHEY_COMPLEX, 0.6,
        Scalar(255, 255, 255), 0.4);

    //********************显示参与单位logo********************//
    auto crop_logo1 = m_canvas(Rect(30, 815, logo_w, logo_h));
    auto crop_logo2 = m_canvas(Rect(140, 815, logo_w, logo_h));
    logo_sia.copyTo(crop_logo1);
    logo_301.copyTo(crop_logo2);
}

void CvUI::process(cv::Mat &frame)
{
    using namespace cv;
    using namespace std;

    char buf[BUFFER_SIZE];

    //***********获取机器人当前状态*************//
    auto bot = VirtualRobot::get_instance();
    const double ud = bot ? bot->get_bend_ud() : 0;
    const double lr = bot ? bot->get_bend_lr() : 0;
    const double leng = bot ? bot->get_length() : 0;
    const double torq = bot ? bot->get_torque() : 0;
    const double rot = bot ? bot->get_rotation() : 0;

    auto &detect = recog::CvDetector::get_instance();
    double img_quality = CvVideo::get_instance().get_quality();

    auto labels = detect.get_labels();

    //***********显示目标检测*************//
    frame = module::utils::draw_bboxes(frame, labels.extra.targets,
        decltype(recog::CvDetector::m_detector)::classes, { 0, 0, 320, 320 });

    //***********视频显示*************//
    auto canvas = m_canvas.clone();
    {
        cv::resize(frame, frame, BFrame.size());
        frame.copyTo(canvas(BFrame));
    }

    //********************异常操作辨识********************//
    if (labels.ope_valid_score > 0.5) {
        putText(canvas, "Normal operation", cv::Point(TAG_L, TAG_T2 + 30), cv::FONT_HERSHEY_COMPLEX,
            0.5, cv::Scalar(0, 255, 0), 0.4, 8);
    } else {
        putText(canvas, "Abnormal operation", cv::Point(TAG_L, TAG_T2 + 30),
            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 255), 0.4, 8);
        auto crop_warning = canvas(Rect(350, 180, logo_w, logo_h));
        img_warning.copyTo(crop_warning);
    }

    //***********显示操作活动辨识结果及工作空间*************//
    {
        static const auto ActiveWSColor = Scalar(0, 255, 0);
        const Point WSCenter(WS_Xc, WS_Yc);

        // 显示操作名称
        auto name = recog::ActionMapper.at(labels.ope_case).full;
        putText(canvas, name, cv::Point(TAG_L, TAG_T1 + 30), cv::FONT_HERSHEY_COMPLEX, 0.5,
            cv::Scalar(0, 255, 0), 0.4, 8);

        // 显示解剖区域
        auto crop_digest = canvas(OpeImg);
        image_digest[int(labels.ope_case)].copyTo(crop_digest);

        // 区域背景范围
        auto lt = cv::Point(WS_Xc - WS.x - WS_Pr, WS_Yc - WS.y - WS_Pr);
        auto rb = cv::Point(WS_Xc + WS.X + WS_Pr, WS_Yc + WS.Y + WS_Pr);
        rectangle(m_canvas, lt, rb, Scalar(255, 255, 255), -1, 2 * WS_Pr);

        // 绘制约束工作区域
        auto wsinfo = recog::ActionMapper.at(labels.ope_case).workspace;
        wsinfo.center.y *= -1; // NOTE: reverse y axis from its real meaning.
        wsinfo.size.x = wsinfo.size.x + 4 * WS_Pr; // Add offset for point visual.
        wsinfo.size.y = wsinfo.size.y + 4 * WS_Pr;

        switch (wsinfo.type) {
        case recog::All:
            // 空间全释放
            rectangle(canvas, lt, rb, ActiveWSColor, -1);
            break;
        case recog::Rectangle:
            rectangle(canvas, WSCenter + wsinfo.center - wsinfo.size / 2,
                WSCenter + wsinfo.center + wsinfo.size / 2, ActiveWSColor, -1);
            break;
        case recog::Ellipse:
            ellipse(
                canvas, WSCenter + wsinfo.center, wsinfo.size / 2, 0, 0, 360, ActiveWSColor, -1);
            break;
        default:
            break;
        }
    }

    //********************弯曲坐标点********************//
    circle(canvas, cv::Point(WS_Xc + lr, WS_Yc - ud), 2 * WS_Pr, CV_RGB(255, 0, 0), -1);
    line(canvas, cv::Point(WS_Xc - WS.x, WS_Yc), cv::Point(WS_Xc + WS.X, WS_Yc), Scalar(0, 0, 0), 1,
        cv::LINE_AA); // line X
    line(canvas, cv::Point(WS_Xc, WS_Yc - WS.y), cv::Point(WS_Xc, WS_Yc + WS.Y), Scalar(0, 0, 0), 1,
        cv::LINE_AA); // line Y

    //********************显示弯曲角度数值********************//
    double output_data[4] = {
        std::clamp<double>(ud, 0, 360),
        std::clamp<double>(-ud, 0, 360),
        std::clamp<double>(lr, 0, 360),
        std::clamp<double>(-lr, 0, 360),
    };
    string output_data1[4];
    for (int i = 0; i < 4; i++) {
        output_data1[i] = DoubleToString(std::round(output_data[i]));
        putText(canvas, output_data1[i], cv::Point(TAG_VALUE_L, TAG_H0 + (i * 70)),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 0.4, 8);
    }

    //********************显示介入力********************//

    Rect rect = Rect(SP_X - 3, SP_Y - 3, EP_X - SP_X + 7, EP_Y - SP_Y + 6);
    double Insertion_froce = 420 * std::clamp(std::abs(torq / 5.0), 0., 1.);
    rectangle(canvas, rect, CV_RGB(255, 255, 255), 2);

    for (int i = 0; i <= Insertion_froce; i++) {
        line(canvas, cv::Point(SP_X + i, SP_Y), cv::Point(SP_X + i, EP_Y),
            CV_RGB((i / 200.0) * 255, (1 - i / 400.0) * 255, 0), 2, cv::LINE_AA);
    }
    line(canvas, cv::Point(SP_X + 100, SP_Y - 1), cv::Point(SP_X + 100, EP_Y + 1),
        CV_RGB(255, 255, 255), 3, cv::LINE_AA);
    line(canvas, cv::Point(SP_X + 250, SP_Y - 1), cv::Point(SP_X + 250, EP_Y + 1),
        CV_RGB(255, 255, 255), 3, cv::LINE_AA);

    //*******************输送长度********************//
    Rect rect2 = Rect(SP_X2 - 3, SP_Y2 - 3, EP_X2 - SP_X2 + 7, EP_Y2 - SP_Y2 + 6);

    // double percent = (g_iGas_length - 150) * 100.0 / (960 - 150);
    double Length = 420 * std::clamp(leng / 1000.0, 0., 1.);
    rectangle(canvas, rect2, CV_RGB(255, 255, 255), 2);

    for (int i = 0; i <= Length; i++) {
        line(canvas, cv::Point(SP_X2 + i, SP_Y2), cv::Point(SP_X2 + i, EP_Y2),
            CV_RGB((i / 200.0) * 255, (1 - i / 400.0) * 255, 0), 2, cv::LINE_AA);
    }
    line(canvas, cv::Point(SP_X2 + 100, SP_Y2 - 1), cv::Point(SP_X2 + 100, EP_Y2 + 1),
        CV_RGB(255, 255, 255), 3, cv::LINE_AA);
    line(canvas, cv::Point(SP_X2 + 250, SP_Y2 - 1), cv::Point(SP_X2 + 250, EP_Y2 + 1),
        CV_RGB(255, 255, 255), 3, cv::LINE_AA);

    //********************显示介入状态********************//
    if (bot && bot->is_forwarding()) {
        putText(canvas, "Advance", cv::Point(TAG_L, TAG_T3 + 30), cv::FONT_HERSHEY_COMPLEX, 0.6,
            cv::Scalar(0, 255, 0), 0.4, 8);
    } else {
        putText(canvas, "Withdrawal", cv::Point(TAG_L, TAG_T3 + 30), cv::FONT_HERSHEY_COMPLEX, 0.6,
            cv::Scalar(0, 0, 255), 0.4, 8);
    }

    //*******************显示图像清晰度********************//
    if (img_quality >= 0.82) {
        putText(canvas, "Clear", cv::Point(TAG_L, TAG_T4 + 30), cv::FONT_HERSHEY_COMPLEX, 0.6,
            Scalar(0, 255, 0), 0.4);
    } else {
        putText(canvas, "Blur", cv::Point(TAG_L, TAG_T4 + 30), cv::FONT_HERSHEY_COMPLEX, 0.6,
            Scalar(0, 0, 255), 0.4);
    }

    //***********显示机器人在线状态*************//
    if (bot->is_online()) {
        cv::circle(canvas, { 1420, 20 }, 6, cv::Scalar(0, 255, 0), -1);
    } else {
        cv::circle(canvas, { 1420, 20 }, 6, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow(m_name, canvas);
    cv::waitKey(1);
}

void CvUI::on_update(cv::Mat &src) {}

void CvUI::on_start() { reload_count++; }

void CvUI::on_close() { cv::destroyWindow(m_name); }
