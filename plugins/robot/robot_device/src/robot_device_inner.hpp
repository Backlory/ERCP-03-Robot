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
        std::uint32_t BeckhoffFollowDataResult(
            unsigned long length, const void *data) const override;
        bool BeckhoffArmOperation(beckhoff_arm_operation iArmOper)const override;
        bool BeckhoffReadAsexPos(double asex_pos[19])const override;
        double BeckhoffForce(INT16 iPos)const override;

        bool IsOpen() const override;
        device::beckhoff::BeckhoffSnapshot BeckhoffSnapshot() const override;

        // ERCP
        bool BeckhoffERCPOperateState(bool state) const override;
        bool BeckhoffIsERCPOnline() const override;
        bool BeckhoffIsERCPReady() const override;

    private:
        void InitDevices(int verbose);

    private:
        _RobotDevice();
        _RobotDevice(const _RobotDevice &) = delete;
        ~_RobotDevice();
    };

} // namespace ercp
