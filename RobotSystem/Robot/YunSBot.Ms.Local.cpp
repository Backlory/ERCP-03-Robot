/**
 * @file YunSBot.Ms.Local。cpp
 * @author ytom (ybb331082@126.com)
 * @brief 机器人主从控制程序(远程版本)
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright (c) 2022 SIA-ILSR
 *
 */
#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <Poco/Net/DatagramSocket.h>
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "YunSBot.h"
#include "RPC/ArmModule.hpp"

namespace ercp {

    extern robot_status GetRobotStatus();

    static const double MasterControlPeriod = 0.008;

    //-------------------------------------------------------------------------

    YunSBot::_master::_master(YunSBot &p)
        : parent(p)
        , client()
    {
        InitMasterSlaveControl();

        // 启动状态发送UDP
        // client.bind(Poco::Net::SocketAddress(31001));
        auto addr = Poco::Net::SocketAddress(GetSettings().Basic.Master(), 31001);
        client.connect(Poco::Net::SocketAddress(GetSettings().Basic.Master(), 31001));
        ROBOT_INFO(GetSettings().Basic.Verbose() > 0,
            fmt::format("Robot bind UDP to {} at {}", addr.toString(), client.address().toString()))

        // 启动命令接收UDP
        Poco::Net::SocketAddress address(31002);
        handlers.push_back(this);
        server = std::make_shared<UdpServer>(handlers, address);
        ROBOT_INFO(GetSettings().Basic.Verbose() > 0,
            fmt::format("Robot bind UDP to {}", server->address().toString()))
    }

    // void YunSBot::_master::StartThreads()
    //{
    //    if (!m_ms_worker) {
    //        m_ms_worker = boost::make_shared<boost::thread>([this]() {
    //            ROBOT_THREADNAME("MsCtrl");
    //
    //            while (!boost::this_thread::interruption_requested()) {
    //                ////////////////////////////////////////////////////////////////////////
    //                double t = ilsr::Time::wall_time();
    //                if (parent.base.IsRobotRunning()) {
    //                    OnMsControl(t);
    //                }
    //                double te = ilsr::Time::wall_time();
    //                ////////////////////////////////////////////////////////////////////////
    //                const auto &per = MasterControlPeriod;
    //                double sleep = std::clamp<double>(per - (te - t), 0, per);
    //                ilsr::Time::sleep_for(sleep);
    //                boost::this_thread::interruption_point();
    //            }
    //        });
    //    }
    //}

    // void YunSBot::_master::ExitThreads()
    //{
    //    if (m_ms_worker) {
    //        m_ms_worker->interrupt();
    //        m_ms_worker->join();
    //        m_ms_worker.reset();
    //    }
    //}

    void YunSBot::_master::processData(char *buf)
    {
        auto size = this->payloadSize(buf);
        control_cmd cmd;
        if (size == sizeof(control_cmd)) {
            const char *buffer = payload(buf);
            std::copy(buffer, buffer + size, (char *)&cmd);
            m_cmd.Set(cmd);
        }
    }

    void YunSBot::_master::processError(char *buf)
    {
        //
    }

    //-------------------------------------------------------------------------

    bool YunSBot::_master::IsMasterOnline() const
    {
        control_cmd cmd;
        return GetMasterCmd(cmd, 0.1);
    }

    bool YunSBot::_master::GetMasterCmd(control_cmd &cmd, double overtime) const
    {
        return m_cmd.TryGet(cmd, overtime);
    }

    void YunSBot::_master::InitMasterSlaveControl()
    {
        //// 主从控制线程
        // OnMsControl.connect(
        //    boost::bind(&YunSBot::_master::MasterRunnable, this, boost::placeholders::_1));
    }

    int YunSBot::_master::SendStatus(const std::vector<char> &data)
    {
        try {
            return client.sendBytes((void *)data.data(), data.size());
        } catch (std::exception e) {
            std::cout << e.what() << std::endl;
        }
        return 0;
    }

} // namespace ercp
