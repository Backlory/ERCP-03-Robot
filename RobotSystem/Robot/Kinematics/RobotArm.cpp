/**
 * @file RobotArm.h
 * @author ytom(杨涛)(yangtao1@sia.cn)
 * @brief 机器人运动学参数
 * @version 0.1
 * @date 2021-03-19
 *
 * @copyright Copyright (c) 2021
 *
 */

#define _USE_MATH_DEFINES
#include <algorithm>
#include <assert.h>
#include <float.h>
#include <math.h>

#include "RobotArm.h"

namespace ercp {

    const double RobotArm::DEG2RAD = (0.017453292519943295);

    const double RobotArm::L1 = 0.17;
    const double RobotArm::L2 = 0.27;
    const double RobotArm::L3 = 0.17;

    bool RobotArm::ForwardKinematics(const double q[3], double& x, double& y, const int& base, const int& target) {

        x = 0, y = 0;

        assert(base <= target);

        int i = base;
        while (i < target) {
            ++i;
            switch (i) {
            case 1:
                x += L1 * cos(q[0]);
                y += L1 * sin(q[0]);
                break;
            case 2:
                x += L2 * cos(q[0] + q[1]);
                y += L2 * sin(q[0] + q[1]);
                break;
            case 3:
                x += L3 * cos(q[0] + q[1] + q[2]);
                y += L3 * sin(q[0] + q[1] + q[2]);
                break;
            default: break;
            }
        }
        return true;
    }

    bool RobotArm::InverseKinematics(
        const double& x, const double& y, const double& theta, double qout[3], bool degree, bool result2) {

        double q[3];
        double ori = theta;
        if (degree) { ori *= DEG2RAD; }
        double A1 = -L3 * cos(ori) + x;
        double B1 = -L3 * sin(ori) + y;
        // 利用平方和与和差化积公式求解方程: { (A1 - L1*cos(q1))^2 + (B1 - L1*sin(q1))^2 = L2^2 }
        double th = atan2(A1, B1);
        double sin_q1th = -(L2 * L2 - L1 * L1 - A1 * A1 - B1 * B1) / (2 * L1 * sqrt(A1 * A1 + B1 * B1));
        if (abs(sin_q1th) > 1) { return false; }
        double q1th = asin(sin_q1th);
        if (!result2) { q1th = M_PI - q1th; }
        q[0] = q1th - th;
        double A2 = A1 - L1 * cos(q[0]);
        double B2 = B1 - L1 * sin(q[0]);
        double q1q2 = atan2(B2, A2);
        q[1] = q1q2 - q[0];
        q[2] = ori - q[0] - q[1];
        qout[0] = q[0];
        qout[1] = q[1];
        qout[2] = q[2];
        return true;
    }

    bool RobotArm::InverseKinematics(
        const double& x, const double& y, const double& theta, std::vector<double>& q, bool degree, bool result2) {

        double out[3];
        if (InverseKinematics(x, y, theta, out, degree, result2)) {
            q.clear();
            q.push_back(out[0]);
            q.push_back(out[1]);
            q.push_back(out[2]);
            return true;
        }
        return false;
    }

} // namespace ercp
