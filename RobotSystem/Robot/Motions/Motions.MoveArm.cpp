#include "cml_driver.hpp"
#include "robot_config.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "yunsbot_config.h"

#include "../YunSBot.h"
#include "../Kinematics/RobotArm_v2.h"

namespace ercp {

    bool PrepareForFollow()
    {
        //??  删除在跟随与open状态切换时，   这个部分代码需要根据倍福的情况进行相应的处理
        //auto m = GetRobot().GetMotor(motor_t::feed_move);
        //if (!m) {
        //    return 0;
        //}
        //if (!m->api.SetPositionLoad(0)) {
        //    return false;
        //}

        //auto &robot = YunSBot::GetInstance().context;
        //// robot.following_complement = 0.0;
        //robot.following_complement_master = 0.0;
        //// robot.following_complement_autofollow = 0.0;
        //robot.following_complement_force = 0.0;

        // 需要在这里判断是否可以跟随
        auto& robot = GetRobot();

        if (beckhoff_arm_move_state::BAMS_OPENED != robot.BeckhoffArmMoveState() &&
            beckhoff_arm_move_state::BAMS_FOLLOWED != robot.BeckhoffArmMoveState() && 
            beckhoff_arm_move_state::BAMS_FOLLOWING != robot.BeckhoffArmMoveState() && 
            beckhoff_arm_move_state::BAMS_FOLLOWED_ONE != robot.BeckhoffArmMoveState()) return false;

        robot.BeckhoffArmOperation(beckhoff_arm_operation::BAO_FOLLOW);

        return true;
    }

    bool PrepareStopFollow() {
        // 判断倍福状态为41时，为跟谁完成，并初始化位置，可以转化为展开状态
        auto &robot = GetRobot();

        if (beckhoff_arm_move_state::BAMS_FOLLOWED != robot.BeckhoffArmMoveState()) return false;

        robot.BeckhoffArmOperation(beckhoff_arm_operation::BAO_OPEN);
        return true;
    }

#define PASS_FEED_ENCODER_CTSPR (512.0) //被动轮一圈码盘
    //#define PASS_NOMINAL_RADIUS     (19.65f)
    //#define ACTI_FEED_ENCODER_CTSPR (MFm_CTSPDEG * 360)
    //#define ACTI_NOMINAL_RADIUS     (12.6f)

    double GetPassiveFeedLength(int encoder)
    {
        auto& yunsbot = YunSBot::GetInstance();
        auto& passive_encoder_offset = yunsbot.context.pas_enc_ofs;
        
        double pos = 0;
        pos = encoder;
        double r = GetSettings().Device.Follow.WheelRadius();
        float p = ((pos - passive_encoder_offset) / PASS_FEED_ENCODER_CTSPR) * r * M_PI * 2 ;
        return -p;
    }

    // double GetActiveFeedLength()
    //{
    //    auto m = GetRobot().GetMotor(motor_t::feed_move);
    //    if (!m)
    //        return 0;
    //    double pos = m->GetPosition();
    //    return (-pos / 360.0) * ACTI_NOMINAL_RADIUS * ACTI_NOMINAL_RADIUS * M_PI * 2;
    //}

    double GetFeedLength(bool disable_encoder,int encoder[5])
    {
        auto &yunsbot = YunSBot::GetInstance();
        int test = encoder[0];
        return GetPassiveFeedLength(test) * (!disable_encoder)
            + yunsbot.context.following_complement_master
            + yunsbot.context.following_complement_autofollow //
            + yunsbot.context.following_complement_force;//单位为mm
    }

    double GetFeedTotal()
    {
        if (GetSettings().Device.ScopeType() == ScopeType_t::Olympus_H260) {
            return 1.000;
        } else {
            return 0.800;
        }
    }

    double GetFollowStart()
    {
        if (GetSettings().Device.ScopeType() == ScopeType_t::Olympus_H260) {
            return 0.5826;
        } else {
            return 0.3826;
        }
    }

    int GetTypeOfScope()
    {
        if (GetSettings().Device.ScopeType() == ScopeType_t::Olympus_H260) {
            return 1;
        } else {
            return 2;
        }
    }
    /// <summary>
    /// 计算跟随轨迹逆运动学
    /// </summary>
    /// <param name="length">轨迹长度坐标</param>
    /// <param name="stored">参考关节位置, 单位:度</param>
    /// <returns></returns>
    const Vector3d Trajectory(const double &length, const Vector3d &stored)
    {
        // 输送总长度[m]
        const double FEED_TOTAL = GetFeedTotal();
        // 操作端开始跟踪时的输送长度[m]
        const double FEED_FOLLOW_START = GetFollowStart();
        // 镜子类型[m]
        const double TYPE_OF_SCOPE = GetTypeOfScope();
        // 圆弧段半径[m]
        double RArc;
        // 直线段长度[m]
        double Length;

        RArc = 0.2371;//GetSettings().Device.Follow.FollowRadius();
        Length = 0.2312;//GetSettings().Device.Follow.FollowLength();

        const auto end_x = ercp::RobotArm::end[0];
        const auto end_y = ercp::RobotArm::end[1];
        // std::cout << end_y << std::endl;
        double x = 0, y = 0, th = 0;
        const auto len = std::clamp<double>(length, 0, FEED_TOTAL); // [m]

        std::vector<double> targetPos(stored.data(), stored.data() + 3);

        if (len >= FEED_TOTAL - Length) {
            // 直线段
            x = end_x + FEED_TOTAL - len;
            y = end_y;
            th = -M_PI * ercp::RobotArm::RAD2DEG;
        } else {
            // 圆弧段
            const double percent = std::clamp<double>(
                (len - FEED_FOLLOW_START) / (FEED_TOTAL - Length - FEED_FOLLOW_START), 0, 1);
            const double origin_x = end_x + Length;
            const double origin_y = end_y + RArc;

            x = origin_x + RArc * std::cos(percent * M_PI_2);
            y = origin_y - RArc * std::sin(percent * M_PI_2);
            th = -(percent * M_PI_2 + M_PI_2) * ercp::RobotArm::RAD2DEG;
        }

        // 使用角度模式
        if (!ercp::RobotArm::InverseKinematics(
                x, y, th, targetPos, targetPos, true, TYPE_OF_SCOPE)) {
            ROBOT_INFO(
                GetSettings().Basic.Verbose(), fmt::format("Trajectory failed at %f.\n", len));
        }

        return std::move(Vector3d(targetPos.data()));
    }

} // namespace ercp
