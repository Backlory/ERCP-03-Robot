/**
 * @file RobotArm.cpp
 * @author ytom(杨涛)(yangtao1@sia.cn)
 * @brief 机器人运动学参数
 * @version 0.1
 * @date 2021-03-19
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __ROBOTARM__
#define __ROBOTARM__

#error "RobotArm is deprecated. Use RobotArm_v2 instead."

#include <vector>

namespace ercp {

    class RobotArm {
    public:
        static const double DEG2RAD;

    public:
        /**
         * @brief 给定线长度，计算悬链线悬挂位置
         *
         * @param len 目标悬链线长度
         * @param x [Out]返回x位置坐标
         * @param y [Out]返回y位置坐标
         */
        static void CatenaryPose(const double &len, double &x, double &y);

        /**
         * @brief 正运动学计算
         *
         * @param q 关节坐标(3个关节)
         * @param x [Out]返回x位置坐标
         * @param y [Out]返回y位置坐标
         * @param base 基坐标系[0~4]，4表示末端
         * @param target 目标坐标系[0~4]，4表示末端
         * @return true
         * @return false
         */
        static bool
        ForwardKinematics(const double q[3], double &x, double &y, const int &base = 0, const int &target = 4);

        /**
         * @brief 逆运动学解算
         *
         * @param x x位置坐标
         * @param y y位置坐标
         * @param theta 姿态角度，以x轴正方向为0°，逆时针为正
         * @param q [Out]返回关节坐标
         * @param degree 是否使用角度(默认：否)
         * @param result2 选择返回另一组解(默认：否)
         * @return true
         * @return false
         */
        static bool InverseKinematics(const double &x,
                                      const double &y,
                                      const double &theta,
                                      double        qout[3],
                                      bool          degree  = false,
                                      bool          result2 = false);

        static bool InverseKinematics(const double &       x,
                                      const double &       y,
                                      const double &       theta,
                                      std::vector<double> &q,
                                      bool                 degree  = false,
                                      bool                 result2 = false);

    public:
        // 2臂长[m]
        static const double L1;
        // 3臂长[m]
        static const double L2;
        // 4臂长[m]
        static const double L3;
        // 镜把长(旋钮中心到镜线末端)[m]
        static const double Lb;
    };

}  // namespace ercp

#endif  // __ROBOTBASE__
