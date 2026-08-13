#include "robot_devices.h"
#include "yunsbot_config.h"

namespace ercp {

/**
 * @brief 检查机械臂是否处于可进入跟随模式的状态并发起跟随动作。
 * @details 只有打开、已跟随或正在跟随状态允许继续；其他状态直接拒绝，避免跨越非法状态转移。
 */
bool PrepareForFollow()
{
    auto &robot = GetRobot();

    if (beckhoff_arm_move_state::BAMS_OPENED != robot.BeckhoffArmMoveState() &&
        beckhoff_arm_move_state::BAMS_FOLLOWED != robot.BeckhoffArmMoveState() &&
        beckhoff_arm_move_state::BAMS_FOLLOWING != robot.BeckhoffArmMoveState()) {
        return false;
    }

    robot.BeckhoffArmOperation(beckhoff_arm_operation::BAO_FOLLOW);
    return true;
}

/**
 * @brief 检查机械臂是否已完成跟随并请求回到打开状态。
 * @details 只有已跟随状态允许退出，其他状态保持原状态并返回失败。
 */
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
