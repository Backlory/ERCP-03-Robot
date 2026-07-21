#include <eigen/Eigen>
#include "yunsbot_config.h"
#include "Settings.hpp"
#include "robot_config.h"
#include "CV/recog.hpp"
#include "Robot/Ms.hpp"

static const double MasterControlPeriod = 0.008;

// static const int api = 7998;
// static const int status = 31001;
// static const int cmd = 31002;
// static const int inject = 31006;

LingMiServer::LingMiServer(Poco::Net::SocketAddress address)
{
    handlers.push_back(this);
    server = std::make_shared<UdpServer>(handlers, address);
    ROBOT_INFO(true, fmt::format("Robot bind UDP to {}", server->address().toString()))
}

void LingMiServer::processData(char *buf)
{
    auto size = this->payloadSize(buf);
    robot_status cmd;
    if (size == sizeof(robot_status)) {
        const char *buffer = payload(buf);
        std::copy(buffer, buffer + size, (char *)&cmd);
        m_status.Set(cmd);
    }
}

void LingMiServer::processError(char *buf)
{
    //
}

//-----------------------------------------------------------------------------

LingMi::LingMi() { VirtualRobot::m_instance = this; }

void LingMi::run()
{
    // 启动状态接收UDP
    Poco::Net::SocketAddress address(Settings::Get().Port());
    server = std::make_shared<LingMiServer>(address);

    // 启动命令发送UDP
    auto addr = Poco::Net::SocketAddress(Settings::Get().Robot(), Settings::Get().Port2());
    client.connect(addr);
    ROBOT_INFO(true,
        fmt::format("Robot bind UDP to {} at {}", addr.toString(), client.address().toString()))
}

void LingMi::close()
{
    client.close();
    server.reset();
}

bool LingMi::is_online() const
{
    robot_status sta;
    return GetStatus(sta, 0.1);
}

bool LingMi::GetStatus(robot_status &status, double overtime) const
{
    return server && server->m_status.TryGet(status, overtime);
}

bool LingMi::SendCmd(const control_cmd &cmd)
{
    try {
        // 序列化
        static std::vector<char> buffer(sizeof(cmd));
        std::copy((char *)&cmd, (char *)&cmd + buffer.size(), buffer.data());
        return client.sendBytes((void *)buffer.data(), buffer.size());
    } catch (std::exception e) {
        std::cout << e.what() << std::endl;
    }
    return false;
}

//-----------------------------------------------------------------------------

double LingMi::get_bend_ud()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        return -sta.pos_bend_ud;
    }
    return 0;
}

double LingMi::get_bend_lr()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        return sta.pos_bend_lr;
    }
    return 0;
}

double LingMi::get_rotation()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        return sta.pos_rotate;
    }
    return 0;
}

double LingMi::get_torque()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        // See Motions.Cycles.cpp, line 49
        return std::abs(sta._reserved[0]); // Use feed force
    }
    return 0;
}

double LingMi::get_length()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        return sta.follow_length;
    }
    return 0;
}

double LingMi::get_q2()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        // See Motions.Cycles.cpp, line 147-159
        return sta._reserved[26];
    }
    return 0;
}

double LingMi::get_q3()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        // See Motions.Cycles.cpp, line 147-159
        return sta._reserved[27];
    }
    return 0;
}

double LingMi::get_q4()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        // See Motions.Cycles.cpp, line 147-159
        return sta._reserved[28];
    }
    return 0;
}

bool LingMi::is_forwarding()
{
    robot_status sta;
    if (this->GetStatus(sta)) {
        return sta.vel_move < 0;
    }
    return false;
}

bool LingMi::ws_constraints(recog::operation_tag_t operation, int isfoward, double force,
    double image_quality, control_cmd &cmd)
{
    using namespace Eigen;

    // 绘制约束工作区域
    auto wsinfo = recog::ActionMapper.at(operation).workspace;

    bool ret = true;

    // 目标运动方向
    const Vector2d aim(cmd.vel_bend_lr, -cmd.vel_bend_ud);
    const Vector2d aimn = aim.normalized();
    // 当前位置
    const Vector2d now(get_bend_lr(), get_bend_ud());
    const Vector2d center(wsinfo.center.x, wsinfo.center.y);
    // 当前位置矢量
    const Vector2d vect = now - center;
    const Vector2d dire = vect.normalized();

    if (cmd.home_bend_lr) {
        // 约束到约束中心
        ret = true;
        // if (wsinfo.type != recog::Disable) {
        //} else
        if (wsinfo.type != recog::All) {
            cmd.vel_bend_lr = std::clamp(-vect[0] / 10, -1., 1.);
            cmd.vel_bend_ud = std::clamp(vect[1] / 10, -1., 1.);
            cmd.home_bend_lr = false;
        } else {
            // use default homing.
        }

    } else {
        if (wsinfo.type == recog::Ellipse) {
            // 计算椭圆切线方向
            // 方程：x^2/a^2 + y^2/b^2 = 1
            const double a = wsinfo.size.x / 2; // 半长轴
            const double b = wsinfo.size.y / 2; // 半短轴
            // 沿着direct方向的边界点, N=(a*cos(al), b*sin(al))
            const Vector2d edge = { a * dire[0], b * dire[1] };
            // 中心沿着direct指向椭圆外侧的法线, N=(b*cos(al), a*sin(al))
            Vector2d normal = { b * dire[0], a * dire[1] };
            normal.normalize();
            // 没有运动
            bool no_motion = aim.norm() < 0.01;
            bool inside_edge = vect.norm() <= edge.norm();
            // 边界外: 允许向内，不许向外（即应该位置方向和运动方向差大于180度）
            if (!no_motion && !inside_edge && normal.dot(aimn) >= 0) {
                ret = false;
            }

        } else if (wsinfo.type == recog::Rectangle) {
            // NotImplemented ...
        } else {
            // NotImplemented ...
        }
    }

    if (!ret) {
        cmd.vel_bend_lr = 0;
        cmd.vel_bend_ud = 0;
    }
    return ret;
}