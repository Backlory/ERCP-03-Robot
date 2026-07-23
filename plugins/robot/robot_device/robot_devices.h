#pragma once
#include <basetsd.h>
#include <memory>
#include <string>
#include "utils.h"
#include "robot_config.h"
#include "beckhoff_snapshot.hpp"
#include "yunsbot_config.h"

namespace ercp {

    class RobotDevice {
    public:
        // 力传感器
        virtual double GetHandleForce(double overtime = 0.1) const = 0;
        virtual double GetScopeForce(double overtime = 0.1) const = 0;
        virtual double GetScopeTorque(double overtime = 0.1) const = 0;
        virtual double GetCannulaForce(double overtime = 0.1) const = 0;
        virtual double GetWireForce(double overtime = 0.1) const = 0;

        // 倍福

        // 展开或折叠
        virtual bool BeckhoffMoveArmTo(bool bIsOpen) const = 0;
        virtual beckhoff_arm_move_state BeckhoffArmMoveState() const = 0;

        // 跟随数据
        virtual std::uint32_t BeckhoffFollowDataResult(
            unsigned long length, const void *data) const = 0;
        virtual bool BeckhoffArmOperation(beckhoff_arm_operation iArmOper)const = 0;
        virtual bool BeckhoffReadAsexPos(double asex_pos[19])const = 0;
        virtual bool IsOpen() const = 0;
        virtual device::beckhoff::BeckhoffSnapshot BeckhoffSnapshot() const = 0;
        virtual double BeckhoffForce(INT16 iPos)const = 0;

        //ERCP
        virtual bool BeckhoffERCPOperateState(bool state) const = 0;
        virtual bool BeckhoffIsERCPOnline() const = 0;
        virtual bool BeckhoffIsERCPReady() const = 0;
    };

    ROBOT_API_MEMBER const std::wstring GetIOInputConfig(int index);
    ROBOT_API_MEMBER const std::wstring GetIOOutputConfig(int index);

#pragma region api
    ROBOT_API_MEMBER RobotDevice &GetRobot();
    ROBOT_API_MEMBER std::string GetLogPath();
#pragma endregion

} // namespace ercp

