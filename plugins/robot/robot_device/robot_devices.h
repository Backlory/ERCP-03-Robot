#pragma once
#include <basetsd.h>
#include <memory>
#include <string>
#include "utils.h"
#include "robot_config.h"
#include "yunsbot_config.h"

namespace ercp {

    class RobotDevice {
    public:
    public:
        virtual void CreateExtraLogger(std::string name = "") = 0;
        virtual std::shared_ptr<ilsr::Logger> GetExtraLogger() = 0;
        virtual void ReleaseExtraLogger() = 0;

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
        virtual bool BeckhoffFollowData(unsigned long length, void* data) const = 0;
        virtual bool BeckhoffFollowData_Oneclick(double target, double bigAngle, double smlAngle) const = 0;
        // 基座移动
        virtual bool BeckhoffBaseMove(unsigned long length, void *data) const = 0;
        virtual bool BeckhoffArmOperation(beckhoff_arm_operation iArmOper)const = 0;


        //virtual bool BeckhoffMotorMove(double data[13], int iType,int bIsUpdate) const = 0;
        virtual bool BeckhoffLinearActuator(INT16 data[2])const = 0;

        //virtual bool BeckhoffMotorPara(int iReadPos, double& dActPos, double& dActVel, int& iActCurrent)const = 0;
        //virtual bool BeckhoffMotorPara2(double dActPos[13], double dActVel[13], int iActCurrent[13])const = 0;
        
        ////2-3-2
        //virtual bool BeckhoffReadAsexPos(double asex_pos[21]) const = 0;
        // 2-3-3
        virtual bool BeckhoffReadAsexPos(double asex_pos[19])const = 0;
        virtual bool BeckhoffEncoder(INT32 data[5])const = 0;
        virtual bool BeckhoffSensor(INT16 data[7])const = 0;
        virtual bool IsOpen() const = 0;

        virtual double BeckhoffFollowLength() const = 0;
        virtual bool Beckhoff_Switch(gpio_output_t out_switch)const = 0;
        virtual double BeckhoffForce(INT16 iPos)const = 0;

        virtual double BeckhoffSmallWhell() const = 0;
        virtual double BeckhoffBigWhell() const = 0;
        virtual double BeckhoffSmallWhellCalc() const = 0;
        virtual double BeckhoffBigWhellCalc() const = 0;
        virtual double BeckhoffRotateDegree() const = 0;
        virtual double BeckhoffLifter() const = 0;

        //ERCP
        virtual bool BeckhoffERCPOperateState(bool state) const = 0;
        virtual bool BeckhoffIsERCPOnline() const = 0;
        virtual bool BeckhoffIsERCPReady() const = 0;

        virtual double BeckhoffGetERCPDeliverForce() const = 0;
        virtual double BeckhoffGetERCPGuidwireForce() const = 0;
        virtual double BeckhoffGetERCPDeliverPos() const = 0;
    };

    ROBOT_API_MEMBER const std::wstring GetIOInputConfig(int index);
    ROBOT_API_MEMBER const std::wstring GetIOOutputConfig(int index);

#pragma region api
    ROBOT_API_MEMBER RobotDevice &GetRobot();
    ROBOT_API_MEMBER std::string GetLogPath();
#pragma endregion

} // namespace ercp

