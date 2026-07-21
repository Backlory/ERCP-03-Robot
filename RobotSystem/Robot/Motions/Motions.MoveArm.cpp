#include "robot_devices.h"
#include "yunsbot_config.h"

namespace ercp {

    bool PrepareForFollow()
    {
        auto &robot = GetRobot();

        if (beckhoff_arm_move_state::BAMS_OPENED != robot.BeckhoffArmMoveState()
            && beckhoff_arm_move_state::BAMS_FOLLOWED != robot.BeckhoffArmMoveState()
            && beckhoff_arm_move_state::BAMS_FOLLOWING != robot.BeckhoffArmMoveState()
            && beckhoff_arm_move_state::BAMS_FOLLOWED_ONE != robot.BeckhoffArmMoveState()) {
            return false;
        }

        robot.BeckhoffArmOperation(beckhoff_arm_operation::BAO_FOLLOW);
        return true;
    }

    bool PrepareStopFollow()
    {
        auto &robot = GetRobot();
        if (beckhoff_arm_move_state::BAMS_FOLLOWED != robot.BeckhoffArmMoveState()) {
            return false;
        }

        robot.BeckhoffArmOperation(beckhoff_arm_operation::BAO_OPEN);
        return true;
    }

} // namespace ercp
