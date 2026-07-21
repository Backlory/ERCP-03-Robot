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
#include <cmath>
#include "RobotArm_v2.h"

/*
*   (q1)        (q2)       (q3)
*  ===(L_x)===O===(L1)===O===(L2)===O===(L3)===E
* |
*(L_y)
* |                                               y+ ↑ 
* E                                                    → x+ 
* 
*/
namespace ercp {

    const long double RobotArm::DEG2RAD = (0.0174532925198889);
    const long double RobotArm::RAD2DEG = (57.29577951326093);

    const double RobotArm::L_x      = 0.2312;
    const double RobotArm::L_y      = 0.2371;
    const double RobotArm::L1       = 0.250;  
    const double RobotArm::L2       = 0.250;
    const double RobotArm::L3       = 0.0993;  
    const double RobotArm::L_DS     = 0.310;
    const double RobotArm::L_GS     = 0.300;
    const double RobotArm::end[]    = { -L_x, -L_y };

    inline double clamp_pi(const double &value) {
        double v = fmod(value, M_PI + M_PI);
        v += (v < -M_PI) * (M_PI + M_PI) - (v > M_PI) * (M_PI + M_PI);
        return v;
    }

    bool RobotArm::ForwardKinematics(const double q[3], double &x, double &y, double &theta, const int &base, const int &target) {
        x = L1 * cos(q[0]) + L2 * cos(q[0] + q[1]);
        y = L1 * sin(q[0]) + L2 * sin(q[0] + q[1]);
        theta = q[0] + q[1] + q[2];
        return true;
    }

    bool RobotArm::InverseKinematics(const double &x, const double &y, const double &theta,
        double qout[3], const double qin[3], int type_of_scope)
    {
        double j3_tar_x = 0;
        double j3_tar_y = 0;
        switch (type_of_scope) {
        case 1:
            j3_tar_x = x + (L3 + L_DS) * sin(-theta - M_PI / 2);
            j3_tar_y = y + (L3 + L_DS) * cos(-theta - M_PI / 2);
            break;
        case 2:
            j3_tar_x = x + (L3 + L_GS) * sin(-theta - M_PI / 2);
            j3_tar_y = y + (L3 + L_GS) * cos(-theta - M_PI / 2);
            break;
        default:
            j3_tar_x = x + (L3 + L_DS) * sin(-theta - M_PI / 2);
            j3_tar_y = y + (L3 + L_DS) * cos(-theta - M_PI / 2);
        }

        // 计算关节1
        double a0 = sqrt(j3_tar_x * j3_tar_x + j3_tar_y * j3_tar_y);
        double q0 = atan2(j3_tar_y, j3_tar_x);
        double temp
            = (j3_tar_x * j3_tar_x + j3_tar_y * j3_tar_y + L1 * L1 - L2 * L2) / (2 * L1 * a0);
        double temp1
            = (j3_tar_x * j3_tar_x + j3_tar_y * j3_tar_y - L1 * L1 + L2 * L2) / (2 * L2 * a0);

        if (abs(temp) > 1 || abs(temp1) > 1) {
            return false;
        }

        double q1[3] = { 0 };//解1
        double q2[3] = { 0 };//解2

        q1[0] = -acos(temp) + q0; 
        q1[1] = acos(temp) + acos(temp1);
        q1[2] = theta - q1[0] - q1[1];

        q2[0] = acos(temp) + q0;
        q2[1] = -acos(temp) - acos(temp1);
        q2[2] = theta - q2[0] - q2[1];

        // Compare result 1 and 2 to get the closest to qin
        double norm1 = (qin[0] - q1[0]) * (qin[0] - q1[0]) +
                       (qin[1] - q1[1]) * (qin[1] - q1[1]) +
                       (qin[2] - q1[2]) * (qin[2] - q1[2]);
        double norm2 = (qin[0] - q2[0]) * (qin[0] - q2[0]) +
                       (qin[1] - q2[1]) * (qin[1] - q2[1]) +
                       (qin[2] - q2[2]) * (qin[2] - q2[2]);
        //if (norm1 <= norm2) {
        //    std::move(q1, q1 + 3, qout);
        //} else {
        //    std::move(q2, q2 + 3, qout);
        //}
        std::move(q2, q2 + 3, qout); // 暂时只采用q2
        return true;
    }

    bool RobotArm::InverseKinematics(const double &x, const double &y, const double &theta,
        std::vector<double> &q, std::vector<double> q0, bool degree, int type_of_scope)
    {

        double out[3];
        if (degree) {
            // Convert degree to rad
            for (auto &q : q0) {
                q *= DEG2RAD;
            }
            if (InverseKinematics(x, y, theta * DEG2RAD, out, q0.data(), type_of_scope)) {
                q.clear();
                // Convert rad ot degree
                q.push_back(out[0] / DEG2RAD);
                q.push_back(out[1] / DEG2RAD);
                q.push_back(out[2] / DEG2RAD);
                return true;
            }
        } else {
            if (InverseKinematics(x, y, theta, out, q0.data(), type_of_scope)) {
                q.clear();
                q.push_back(out[0]);
                q.push_back(out[1]);
                q.push_back(out[2]);
                return true;
            }
        }
        return false;
    }

}  // namespace ercp
