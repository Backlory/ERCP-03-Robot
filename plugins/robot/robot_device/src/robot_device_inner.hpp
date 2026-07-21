#include <assert.h>
#include "beckhoff_driver.hpp"

#include "../robot_settings.hpp"
#include "../robot_devices.h"
#include <mmsystem.h>


namespace ercp {

    using namespace device;

    ///////////////////////////////////////////////////////////////////////////

    class _RobotDevice : public RobotDevice {
    public:
        static _RobotDevice &GetInstance()
        {
            static _RobotDevice lc;
            return lc;
        }

    public:
        // 力传感器
        double GetHandleForce(double overtime = 0.1) const override;
        double GetScopeForce(double overtime = 0.1) const override;
        double GetScopeTorque(double overtime = 0.1) const override;
        double GetCannulaForce(double overtime = 0.1) const override;
        double GetWireForce(double overtime = 0.1) const override;

        //// 倍福

        // 展开折叠
        bool BeckhoffMoveArmTo(bool bIsOpen) const override;
        beckhoff_arm_move_state BeckhoffArmMoveState() const override;
        // 跟随数据
        bool BeckhoffFollowData(unsigned long length, void* data) const override;
        bool BeckhoffFollowData_Oneclick(double target, double bigAngle, double smlAngle) const override;
        // 基座移动
        bool BeckhoffBaseMove(unsigned long length, void *data) const override;
        bool BeckhoffArmOperation(beckhoff_arm_operation iArmOper)const override;

        //bool BeckhoffMotorMove(double data[13], int iType, int bIsUpdate)const override;
        bool BeckhoffLinearActuator(INT16 data[2])const override;

        //bool BeckhoffMotorPara(int iReadPos, double& dActPos, double& dActVel, int& iActCurrent)const override;
        //bool BeckhoffMotorPara2(double dActPos[13], double dActVel[13], int iActCurrent[13])const override;
        
        //2-3-2
        //bool BeckhoffReadAsexPos(double asex_pos[21]) const override;
        ////2-3-3
        bool BeckhoffReadAsexPos(double asex_pos[19])const override;
        bool BeckhoffEncoder(INT32 data[5])const override;
        bool BeckhoffSensor(INT16 data[7])const override;

        double BeckhoffFollowLength() const override;
        bool Beckhoff_Switch(gpio_output_t out_switch)const override;
        double BeckhoffForce(INT16 iPos)const override;
        double BeckhoffLifter() const override;

        double BeckhoffSmallWhell() const override;
        double BeckhoffBigWhell() const override;
        double BeckhoffRotateDegree() const override;
        
        double BeckhoffSmallWhellCalc() const override;
        double BeckhoffBigWhellCalc() const override;

        bool IsOpen() const override;

        // ERCP
        bool BeckhoffERCPOperateState(bool state) const override;
        bool BeckhoffIsERCPOnline() const override;
        bool BeckhoffIsERCPReady() const override;

        double BeckhoffGetERCPDeliverForce() const override;
        double BeckhoffGetERCPGuidwireForce() const override;
        double BeckhoffGetERCPDeliverPos() const override;
    public:
        void CreateExtraLogger(std::string name) override;
        std::shared_ptr<ilsr::Logger> GetExtraLogger() override;
        void ReleaseExtraLogger() override;

    private:
        void InitDevices(int verbose);

    private:
        std::mutex m_logger_mutex;
        std::shared_ptr<ilsr::Logger> m_extra_logger;

    private:
        _RobotDevice();
        _RobotDevice(const _RobotDevice &) = delete;
        ~_RobotDevice();
    };

} // namespace ercp
