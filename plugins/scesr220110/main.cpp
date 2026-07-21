#include "utils.h"
#include "Settings.hpp"
#include "CV/cvtask.hpp"
#include "CV/recog.hpp"
#include "CV/cvui.hpp"
#include "Robot/Ms.hpp"
#include "win_joystick.hpp"
#include "joystick.truemaster.h"

using namespace ilsr;
using namespace device::joystick;

const double MasterControlPeriod = 0.008;

// 界面消息记录
std::shared_ptr<ilsr::Logger> MsgLogger;
// 主控制线程
std::shared_ptr<boost::thread> m_ThreadControl;

double clmap(double v, double k) { return (std::abs(v) < k) ? 0.0 : v; }

int main()
{
    // 通过调用初始化定时器
    ilsr::Time::GetInstance();
    // 通过调用第一次加载配置
    Settings::Get();

    // 加载网络模型
    std::cout << "Loading models ..." << std::endl;
    recog::CvDetector::get_instance();

    // 通过调用启动记录线程
    if (Settings::Get().Log.MainLog()) {
        auto name = std::string("mainlog-" + ilsr::Time::logtime() + ".log");
        MsgLogger = std::make_shared<ilsr::Logger>(name);
        MsgLogger->AddLog("t, ud, lr, rot, torq, leng, q2, q3, q4, oc, ov, cc, cv, ec, ev\n");
    }

    // 加载机器人接口
    LingMi me;
    me.run();

    // 加载手柄
    JoyStick js1(JOYSTICKID1), js2(JOYSTICKID2);
    js1.Open();
    js2.Open();

    // 加载视频流和UI界面
    std::cout << "Running ..." << std::endl;
    CvVideo::get_instance().run();
    CvUI::get_instance().run();
    recog::CvDetector::get_instance().run();

    m_ThreadControl = std::make_shared<boost::thread>([&]() {
        while (!boost::this_thread::interruption_requested()) {
            ////////////////////////////////////////////////////////////////////////
            double t = ilsr::Time::wall_time();

            if (me.is_online()) {
                // 手柄动作
                joy_data_ex jL, jR;
                if (js1.GetCurrentJoyData(jL) && js2.GetCurrentJoyData(jR)) {

                    // 手柄指令解析
                    control_cmd cmd;
                    cmd.home_bend_lr = jL.Buttons[0] & (int)joy_button_state::i_Keep;
                    cmd.home_bend_ud = cmd.home_bend_lr;
                    cmd.home_rotate = jR.Buttons[0] & (int)joy_button_state::i_Keep;

                    cmd.vel_bend_lr = clmap(jL.Xpos, 0.25);
                    cmd.vel_bend_ud = clmap(jL.Ypos, 0.25);
                    cmd.vel_rotate = clmap(jR.Xpos, 0.25);
                    cmd.vel_move = clmap(jR.Ypos, 0.25);

                    // 约束
                    auto &detect = recog::CvDetector::get_instance();
                    double img_quality = CvVideo::get_instance().get_quality();
                    auto labels = detect.get_labels();

                    me.ws_constraints(
                        labels.ope_case, me.is_forwarding(), me.get_torque(), img_quality, cmd);

                    // 发送指令
                    me.SendCmd(cmd);
                } else {
                    std::cout << "Joystick offline: ";
                    if (jL.timestamp < 0) {
                        std::cout << " <Left> ";
                    }
                    if (jR.timestamp < 0) {
                        std::cout << " <Right> ";
                    }
                    std::cout << std::endl;
                }

                if (MsgLogger) {
                    // Get status
                    auto bot = VirtualRobot::get_instance();
                    const double ud = bot ? bot->get_bend_ud() : 0;
                    const double lr = bot ? bot->get_bend_lr() : 0;
                    const double rot = bot ? bot->get_rotation() : 0;
                    const double torq = bot ? bot->get_torque() : 0;
                    const double leng = bot ? bot->get_length() : 0;
                    const double q2 = bot ? bot->get_q2() : 0;
                    const double q3 = bot ? bot->get_q3() : 0;
                    const double q4 = bot ? bot->get_q4() : 0;

                    auto &detect = recog::CvDetector::get_instance();
                    auto labels = detect.get_labels();

                    auto log = fmt::format(
                        "{:.06f}, {:.06f}, {:.06f}, {:.06f}, {:.06f}, {:.06f}, "
                        "{:.06f}, {:.06f}, {:.06f}, {:d}, {:.03f}, {:d}, {:.03f}, {:d}, {:.03f}\n",
                        t, ud, lr, rot, torq, leng, q2, q3, q4, (int)labels.ope_case,
                        labels.ope_valid_score, (int)labels.extra.curr_label,
                        labels.extra.curr_score, (int)labels.extra.elm_label,
                        labels.extra.elm_score);

                    MsgLogger->AddLog(log);
                }
            }

            double te = ilsr::Time::wall_time();
            ////////////////////////////////////////////////////////////////////////
            const auto &per = MasterControlPeriod;
            double sleep = std::clamp<double>(per - (te - t), 0, per);
            ilsr::Time::sleep_for(sleep);
            boost::this_thread::interruption_point();
        }
    });
    m_ThreadControl->detach();

    while (true) {
        ilsr::Time::sleep_for(0.1);
    }
    return 0;
}
