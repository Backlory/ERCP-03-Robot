/**
 * @file YunSBot.Ms.Remote.cpp
 * @author ytom (ybb331082@126.com)
 * @brief 机器人主从控制程序(远程版本)
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright (c) 2022 SIA-ILSR
 *
 */
#include "YunSBot.h"
#include "RemoteManager.h"

namespace ercp {

    // 定时器周期[ms]
    static const double TIMERNO_CONTROL_PERIOD = 0.007; // 机器人控制周期

    YunSBot::_master::_master(YunSBot &p)
        : parent(p)
    {
        InitMasterSlaveControl();
    }

    bool YunSBot::_master::IsMasterOnline() const
    {
        auto &master = RemoteManager::GetInstance().GetStatusServer();
        return master.is_connected();
    }

    bool YunSBot::_master::GetMasterCmd(master_cmd_t &cmd, double overtime)
    {
        auto &master = RemoteManager::GetInstance().GetStatusServer();
        return master.get_command(cmd, overtime);
    }

    void YunSBot::_master::InitMasterSlaveControl()
    {
        // TODO: 改为主端请求控制再开启线程
        // OnRobotControlCycle.connect([this](double t, bool) {
        //    //
        //    return false;
        //});
    }

    void YunSBot::_master::MasterControlWorker() {}

} // namespace ercp
