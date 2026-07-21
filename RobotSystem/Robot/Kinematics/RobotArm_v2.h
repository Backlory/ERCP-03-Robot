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

#include <vector>

namespace ercp {

    class RobotArm {
    public:
        static const long double DEG2RAD;
        static const long double RAD2DEG;
    public:
        RobotArm() = delete;

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
         * @param theta [Out]返回末端角度
         * @param base 基坐标系[0~5]，5表示末端
         * @param target 目标坐标系[0~5]，5表示末端
         * @return true
         * @return false
         */
        static bool
        ForwardKinematics(const double q[3], double &x, double &y, double &theta, const int &base = 0, const int &target = 5);

        /**
         * @brief 逆运动学解算
         *
         * @param x x位置坐标
         * @param y y位置坐标
         * @param theta 姿态角度，以x轴正方向为0 rad，逆时针为正  弧度制
         * @param qout [Out]返回关节坐标
         * @param qin [In]起始关节
         * @param degree 是否使用角度(默认：否)
         * @param result2 选择返回另一组解(默认：否)
         * @return true
         * @return false
         */
        static bool InverseKinematics(const double &x,
                                      const double &y,
                                      const double &theta,
                                      double        qout[3],
                                      const double  qin[3]  = NULL,
                                      int           type_of_scope = 1);

        static bool InverseKinematics(const double &       x,
                                      const double &       y,
                                      const double &       theta,
                                      std::vector<double> &q,
                                      std::vector<double>  q0      = std::vector<double>(),
                                      bool degree = false, 
                                      int type_of_scope = 1);

    public:
        // 输送端到轴1水平/垂直距离[m]
        static const double L_x;
        static const double L_y;
        // 1臂长[m]
        static const double L1;
        // 2臂长[m]
        static const double L2;
        // 3臂长[m] （轴3至镜体末端距离)
        static const double L3;
        // 镜长[m]  （拨轮中心至最末刻度线距离)
        static const double L_DS;//奥林巴斯 十二指肠镜
        static const double L_GS;//奥华     胃镜
        // 目标终点位置
        static const double end[2];
    };

}  // namespace ercp

#endif  // __ROBOTBASE__
