/**
 * @file RobotBase.cpp
 * @author ytom(杨涛)(yangtao1@sia.cn)
 * @brief 机器人基座(平面)运动学
 * @version 0.1
 * @date 2021-03-22
 *
 * @copyright Copyright (c) 2021
 *
 *  O===(l1)===O===(l2)===E
 *  ^          ^          ^         ↑ y+
 *  |          |          |    x+ ←
 * (q1)       (q2)       (q3)
 */
#define _USE_MATH_DEFINES
#include <algorithm>
#include <assert.h>
#include <float.h>
#include <math.h>
#include <iostream>
#include <fmt/format.h>
#include "RobotBase.h"

namespace ercp {

#define DEG2RAD (0.017453292519943295)

    const double RobotBase::L1 = 0.25;
    const double RobotBase::L2 = 0.25;
    const double RobotBase::L3_1 = 0.31;
    RobotBase robot_base_kinematics;
    
    inline double clamp_pi(const double &value)
    {
        double v = fmod(value, M_PI + M_PI);
        v += (v < -M_PI) * (M_PI + M_PI) - (v > M_PI) * (M_PI + M_PI);
        return v;
    }

    bool RobotBase::ForwardKinematics(const double q[3], double &x, double &y, double &theta)
    {   
        //定义远心点
        robot_base_kinematics.L3_2 = 0.3322;

        x = L1 * cos(q[0]) + L2 * cos(q[0] + q[1]);
        y = L1 * sin(q[0]) + L2 * sin(q[0] + q[1]);
        theta = q[0] + q[1] + q[2];
        robot_base_kinematics.x_init = x;
        robot_base_kinematics.y_init = y;
        robot_base_kinematics.th_init = theta;

        robot_base_kinematics.L3_th_init = theta + atan2(L3_1, robot_base_kinematics.L3_2);
        robot_base_kinematics.L3_x_init = x + sqrt(L3_1 * L3_1 + robot_base_kinematics.L3_2 * robot_base_kinematics.L3_2) * cos(robot_base_kinematics.L3_th_init);
        robot_base_kinematics.L3_y_init = y + sqrt(L3_1 * L3_1 + robot_base_kinematics.L3_2 * robot_base_kinematics.L3_2) * sin(robot_base_kinematics.L3_th_init);

        //std::cout << fmt::format("{:.06f}, {:.06f}, {:.06f}\n", robot_base_kinematics.x_init, robot_base_kinematics.y_init, robot_base_kinematics.th_init) << std::endl;

        return true;
    }

    bool RobotBase::InverseKinematics(const double &x, const double &y, const double &theta,
        double q[3], const bool &degree, const bool &result2)
    {
        double ori = theta;
        if (degree) {
            ori *= DEG2RAD;
        }
        // 计算关节1
        double a0 = sqrt(x * x + y * y);
        if (a0 < 0.10 * (L1 + L2) || a0 > 0.90 * (L1 + L2)) {
            return false;
        }

        double th = atan2(x, y);
        double sin_alq1th = (x * x + y * y + L1 * L1 - L2 * L2) / (2 * L1 * sqrt(x * x + y * y));
        if (abs(sin_alq1th) > 1) {
            return false;
        }
        double q1th = asin(sin_alq1th);
        if (!result2) {
            q1th = M_PI - q1th;
        }
        q[0] = q1th - th; // q[0] + th = q1th
        // 计算关节2
        double A = x - L1 * cos(q[0]);
        double B = y - L1 * sin(q[0]);
        if (abs(A) <= DBL_EPSILON && abs(B) <= DBL_EPSILON) {
            return false;
        }
        double q1q2 = atan2(B, A);
        q[1] = q1q2 - q[0]; // q[0] + q[1] = q1q2
        //  计算关节3
        q[2] = ori - q[0] - q[1]; // q[0] + q[1] + q[2] = ori
        return true;
    }

    bool RobotBase::InverseKinematics_new(const double &x, const double &y, const double &theta,
        double q_out[3], const double q_orig[3])
    {
        // 计算关节1
        double a0 = sqrt(x * x + y * y);
        double q0 = atan2(y, x);
        double temp = (x * x + y * y + L1 * L1 - L2 * L2) / (2 * L1 * a0);
        double temp1 = (x * x + y * y - L1 * L1 + L2 * L2) / (2 * L2 * a0);

        if (abs(temp) > 1 || abs(temp1) > 1) {
            return false;
        }

        double q1[3] = { 0 }; //解1
        double q2[3] = { 0 }; //解2

        q1[0] = -acos(temp) + q0;
        q1[1] = acos(temp) + acos(temp1);
        q1[2] = theta - q1[0] - q1[1];

        q2[0] = acos(temp) + q0;
        q2[1] = -acos(temp) - acos(temp1);
        q2[2] = theta - q2[0] - q2[1];

        // Compare result 1 and 2 to get the closest to qin
        double norm1 = (q_orig[0] - q1[0]) * (q_orig[0] - q1[0])
            + (q_orig[1] - q1[1]) * (q_orig[1] - q1[1]) + (q_orig[2] - q1[2]) * (q_orig[2] - q1[2]);
        double norm2 = (q_orig[0] - q2[0]) * (q_orig[0] - q2[0])
            + (q_orig[1] - q2[1]) * (q_orig[1] - q2[1]) + (q_orig[2] - q2[2]) * (q_orig[2] - q2[2]);
         if (norm1 <= norm2) {
            q_out[0] = q1[0];
            q_out[1] = q1[1];
            q_out[2] = theta - q1[0] - q1[1]; 
        } else {
            q_out[0] = q2[0];
            q_out[1] = q2[1];
            q_out[2] = theta - q2[0] - q2[1]; 
        }
         //角度限位
        if ((q_out[0] >= 10*3.1415926/180) && (q_out[0] <= 170 * 3.1415926 / 180) && (q_out[1] >= -180 * 3.1415926 / 180) && (q_out[1] <= 180 * 3.1415926 / 180)
            && (q_out[2] <= -20 * 3.1415926 / 180) && (q_out[2] >= -120 * 3.1415926 / 180)) {
            //q_out[0] += offset1;//20°
            //q_out[1] += offset2;//182.9°
            //q_out[2] += offset3;//152°
            q_out[0] = q_out[0];
            q_out[1] = q_out[1];
            q_out[2] = q_out[2];
        }
        else {
            return false;
        }
        //未开启远心时，远心点位置计算时更新
        robot_base_kinematics.L3_th_init = theta + atan2(L3_1, robot_base_kinematics.L3_2);
        if (robot_base_kinematics.start_telecentric == false) {
            robot_base_kinematics.L3_x_init = x + sqrt(L3_1 * L3_1 + robot_base_kinematics.L3_2 * robot_base_kinematics.L3_2) * cos(robot_base_kinematics.L3_th_init);
            robot_base_kinematics.L3_y_init = y + sqrt(L3_1 * L3_1 + robot_base_kinematics.L3_2 * robot_base_kinematics.L3_2) * sin(robot_base_kinematics.L3_th_init);
        }
        else {
            robot_base_kinematics.L3_x_init = robot_base_kinematics.L3_x_init;
            robot_base_kinematics.L3_y_init = robot_base_kinematics.L3_y_init;
        }

        return true;
    }

    bool RobotBase::KeyMoving(double q[3], const double move_dir[3],  const double &deadzone)
    {
        // 3、解算当前位姿，存储于px, py, th；
        const double dv = 0.0005;
        double px, py, th;
        if(robot_base_kinematics.pos_estimate==false){
            if (!ForwardKinematics(q, px, py, th)) {
                std::cout << "ForwardKinematics failed \n";
                return false;
            } // 正运动学失败,当前关键坐标不正确
            robot_base_kinematics.pos_estimate = true;
        }

        if (robot_base_kinematics.pos_estimate == true) {
            // 4、计算位移增量
            double dx = 0, dx_x = 0, dx_y = 0, dx_th = 0, dy = 0, dy_x = 0, dy_y = 0, dy_th = 0,dth = 0;
            double dz = deadzone < 0 ? 0 : (deadzone > 1 ? 1 : deadzone);
            //旋转增量
            if (abs(move_dir[2]) > dz) {
                dth = move_dir[2] > 0 ? dv : -dv;
                robot_base_kinematics.th_init += dth;
                robot_base_kinematics.start_telecentric = true;

               /* if (robot_base_kinematics.th_init >= -M_PI / 4 && robot_base_kinematics.th_init <= atan2(robot_base_kinematics.y_init, robot_base_kinematics.x_init)) {
                    robot_base_kinematics.th_init += dth;
                    robot_base_kinematics.start_telecentric = true;
                }
                else if (robot_base_kinematics.th_init <= -M_PI / 4) {
                    if (dth > 0) {
                        robot_base_kinematics.th_init = robot_base_kinematics.th_init;
                        robot_base_kinematics.start_telecentric = false;
                    }
                    else if (dth <= 0) {
                        robot_base_kinematics.th_init += dth;
                        robot_base_kinematics.start_telecentric = true;
                    }
                }
                else if (robot_base_kinematics.th_init < atan2(robot_base_kinematics.y_init, robot_base_kinematics.x_init)) {
                    if (dth > 0) {
                        robot_base_kinematics.th_init += dth;
                        robot_base_kinematics.start_telecentric = true;
                    }
                    else if (dth <= 0) {
                        robot_base_kinematics.th_init = robot_base_kinematics.th_init;
                        robot_base_kinematics.start_telecentric = false;
                    }
                }*/
            }
            else if (abs(move_dir[2]) <= dz) {
                dth = 0;
                robot_base_kinematics.start_telecentric = false;
            }
            //远心运动位置增量
            if (robot_base_kinematics.start_telecentric == true) {
                robot_base_kinematics.x_init_new= robot_base_kinematics.L3_x_init- sqrt(L3_1 * L3_1 + robot_base_kinematics.L3_2 * robot_base_kinematics.L3_2) * cos(robot_base_kinematics.L3_th_init);
                robot_base_kinematics.y_init_new = robot_base_kinematics.L3_y_init - sqrt(L3_1 * L3_1 + robot_base_kinematics.L3_2 * robot_base_kinematics.L3_2) * sin(robot_base_kinematics.L3_th_init);

                dx_th = robot_base_kinematics.x_init_new - robot_base_kinematics.x_init;
                dy_th = robot_base_kinematics.y_init_new - robot_base_kinematics.y_init;


            }
            else {
                robot_base_kinematics.x_init_new = robot_base_kinematics.x_init;
                robot_base_kinematics.y_init_new = robot_base_kinematics.y_init;
            }
            //平移运动位置增量
            if (abs(move_dir[0]) > dz) {
                dx_x = move_dir[0] > 0 ? dv * cos(robot_base_kinematics.th_init) : -dv * cos(robot_base_kinematics.th_init);
                dy_x = move_dir[0] > 0 ? dv * sin(robot_base_kinematics.th_init) : -dv * sin(robot_base_kinematics.th_init);
            }
            else if (abs(move_dir[0]) <= dz) {
                dx_x = 0;
                dy_x = 0;
            }
            if (abs(move_dir[1]) > dz) {
                dx_y = move_dir[1] > 0 ? -dv * sin(robot_base_kinematics.th_init) : dv * sin(robot_base_kinematics.th_init);
                dy_y = move_dir[1] > 0 ? dv * cos(robot_base_kinematics.th_init) : -dv * cos(robot_base_kinematics.th_init);

            }
            else if (abs(move_dir[1]) <= dz) {
                dx_y = 0;
                dy_y = 0;
            }
            //std::cout << fmt::format(
            //    "{:.06f}, {:.06f}, {:.06f}, {:.06f}, {:.06f}, {:.06f}\n", robot_base_kinematics.x_init, robot_base_kinematics.y_init, robot_base_kinematics.L3_x_init, robot_base_kinematics.L3_y_init, robot_base_kinematics.x_init_new, robot_base_kinematics.y_init_new);

            dx = dx_x + dx_y + dx_th;
            dy = dy_x + dy_y + dy_th;

            // 5、根据位移增量，计算电机绝对转角n_q；
            //运动限位 在一个圆环内（R=L1+L2,r）
            double R = L1 + L2;
            double r = 0.2834; //10°角对应边长
            robot_base_kinematics.x_init += dx;
            robot_base_kinematics.y_init += dy;
            //if (((robot_base_kinematics.x_init * robot_base_kinematics.x_init + robot_base_kinematics.y_init * robot_base_kinematics.y_init) <= R * R * 0.9) && ((robot_base_kinematics.x_init * robot_base_kinematics.x_init + robot_base_kinematics.y_init * robot_base_kinematics.y_init) >= r * r * 0.2)) {
            //    robot_base_kinematics.x_init += dx;
            //    //robot_base_kinematics.y_init += dy; //y轴限位
            //    if (robot_base_kinematics.y_init >= 0) {
            //        robot_base_kinematics.y_init += dy;
            //    }
            //    else {
            //        if (dy > 0) {
            //            robot_base_kinematics.y_init += dy;
            //        }
            //        else {
            //            robot_base_kinematics.y_init += robot_base_kinematics.y_init;
            //        }
            //    }
            //}
            //else if ((robot_base_kinematics.x_init * robot_base_kinematics.x_init + robot_base_kinematics.y_init * robot_base_kinematics.y_init) > R * R * 0.9) {
            //    if (robot_base_kinematics.x_init > 0) {
            //        if ((robot_base_kinematics.x_init + dx) > robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init = robot_base_kinematics.x_init;
            //        }
            //        else if ((robot_base_kinematics.x_init + dx) <= robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init += dx;
            //        }
            //    }
            //    else {
            //        if ((robot_base_kinematics.x_init + dx) < robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init = robot_base_kinematics.x_init;
            //        }
            //        else if ((robot_base_kinematics.x_init + dx) >= robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init += dx;
            //        }
            //    }
            //    if (robot_base_kinematics.y_init > 0) {
            //        if ((robot_base_kinematics.y_init + dy) > robot_base_kinematics.y_init) {
            //            robot_base_kinematics.y_init = robot_base_kinematics.y_init;
            //        }
            //        else if ((robot_base_kinematics.y_init + dy) <= robot_base_kinematics.y_init) {
            //            //robot_base_kinematics.y_init += dy; //y轴限位
            //            if (robot_base_kinematics.y_init >= 0) {
            //                robot_base_kinematics.y_init += dy;
            //            } else {
            //                if (dy > 0) {
            //                    robot_base_kinematics.y_init += dy;
            //                } else {
            //                    robot_base_kinematics.y_init += robot_base_kinematics.y_init;
            //                }
            //            }
            //        }
            //    }
            //    else {
            //        if ((robot_base_kinematics.y_init + dy) < robot_base_kinematics.y_init) {
            //            robot_base_kinematics.y_init = robot_base_kinematics.y_init;
            //        }
            //        else if ((robot_base_kinematics.y_init + dy) >= robot_base_kinematics.y_init) {
            //            //robot_base_kinematics.y_init += dy; //y轴限位
            //            if (robot_base_kinematics.y_init >= 0) {
            //                robot_base_kinematics.y_init += dy;
            //            } else {
            //                if (dy > 0) {
            //                    robot_base_kinematics.y_init += dy;
            //                } else {
            //                    robot_base_kinematics.y_init += robot_base_kinematics.y_init;
            //                }
            //            }
            //        }
            //    }
            //}
            //else if ((robot_base_kinematics.x_init * robot_base_kinematics.x_init + robot_base_kinematics.y_init * robot_base_kinematics.y_init) < r * r * 2) {
            //    if (robot_base_kinematics.x_init > 0) {
            //        if ((robot_base_kinematics.x_init + dx) < robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init = robot_base_kinematics.x_init;
            //        }
            //        else if ((robot_base_kinematics.x_init + dx) >= robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init += dx;
            //        }
            //    }
            //    else {
            //        if ((robot_base_kinematics.x_init + dx) > robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init = robot_base_kinematics.x_init;
            //        }
            //        else if ((robot_base_kinematics.x_init + dx) <= robot_base_kinematics.x_init) {
            //            robot_base_kinematics.x_init += dx;
            //        }
            //    }
            //    if (robot_base_kinematics.y_init > 0) {
            //        if ((robot_base_kinematics.y_init + dy) < robot_base_kinematics.y_init) {
            //            robot_base_kinematics.y_init = robot_base_kinematics.y_init;
            //        }
            //        else if ((robot_base_kinematics.y_init + dy) >= robot_base_kinematics.y_init) {
            //            //robot_base_kinematics.y_init += dy; //y轴限位
            //            if (robot_base_kinematics.y_init >= 0) {
            //                robot_base_kinematics.y_init += dy;
            //            } else {
            //                if (dy > 0) {
            //                    robot_base_kinematics.y_init += dy;
            //                } else {
            //                    robot_base_kinematics.y_init += robot_base_kinematics.y_init;
            //                }
            //            }
            //        }
            //    }
            //    else {
            //        if ((robot_base_kinematics.y_init + dy) > robot_base_kinematics.y_init) {
            //            robot_base_kinematics.y_init = robot_base_kinematics.y_init;
            //        }
            //        else if ((robot_base_kinematics.y_init + dy) <= robot_base_kinematics.y_init) {
            //            //robot_base_kinematics.y_init += dy; //y轴限位
            //            if (robot_base_kinematics.y_init >= 0) {
            //                robot_base_kinematics.y_init += dy;
            //            } else {
            //                if (dy > 0) {
            //                    robot_base_kinematics.y_init += dy;
            //                } else {
            //                    robot_base_kinematics.y_init += robot_base_kinematics.y_init;
            //                }
            //            }
            //        }
            //    }
            //}
            //std::cout << fmt::format(
             //   "{:.06f}, {:.06f}, {:.06f}\n", robot_base_kinematics.x_init, robot_base_kinematics.y_init, sqrt(robot_base_kinematics.y_init* robot_base_kinematics.y_init+ robot_base_kinematics.x_init* robot_base_kinematics.x_init));
            double n_q1[3]; // , n_q2[2];
            if (!InverseKinematics_new(robot_base_kinematics.x_init, robot_base_kinematics.y_init, robot_base_kinematics.th_init, n_q1, q)) {
                std::cout << "InverseKinematics failed \n";
                double px, py, th;
                ForwardKinematics(q, px, py, th);//重置当前位置
                return false;
            } // 求逆运动学失败,新位置不合法
            q[0] = n_q1[0];
            q[1] = n_q1[1];
            q[2] = n_q1[2];
            return true;
        }
        else { 
            return false; 
        }
    }

    bool RobotBase::DragMoving(double q[3], const double force[3], const double &velocity,
        const double &angvel, const double &dt, const double &senseforce, const double &sensetorque,
        const double &deadzone)
    {

        assert(velocity >= 0);
        assert(dt > 0);

        bool ismove = false;
        double dz = deadzone < 0 ? 0 : (deadzone > 1 ? 1 : deadzone);
        double lvel = abs(velocity);
        double avel = abs(angvel);

        double dx, dy, dth;

        // 约束力的范围
        double fx = force[0], fy = force[1], tq = force[2];
        double n_norm = 0;
        if (senseforce > 0) {
            double norm = sqrt(fx * fx + fy * fy);
            if (norm < dz * senseforce) {
                n_norm = 0;
                fx = fy = 0;
            } else {
                n_norm = norm > senseforce ? senseforce : norm;
                fx *= n_norm / norm / senseforce;
                fy *= n_norm / norm / senseforce;
            }
        }
        if (n_norm < DBL_EPSILON) {
            // 无力作用,不移动
            dx = dy = 0;
        } else {
            ismove = ismove || true;
            dx = fx * velocity * dt;
            dy = fy * velocity * dt;
        }

        if (sensetorque > 0) {
            tq = tq < -sensetorque ? -sensetorque : (tq > sensetorque ? sensetorque : tq);
            if (abs(tq) < dz * sensetorque) {
                tq = 0;
            }
            tq /= sensetorque;
        }
        if (abs(tq) < DBL_EPSILON) {
            // 无扭矩,不转动
            dth = 0;
        } else {
            ismove = ismove || true;
            dth = tq * angvel * dt;
        }

        // 无移动
        if (!ismove) {
            return false;
        }

        // 获取当前位姿
        double px, py, th;
        if (!ForwardKinematics(q, px, py, th)) {
            return false;
        } //正运动学失败,当前关键坐标不正确

        // 计算新位姿
        double n_px = px + dx;
        double n_py = py + dy;
        double n_th = th + dth;
        double n_q[3];
        if (!InverseKinematics(n_px, n_py, n_th, n_q, q)) {
            return false;
        } // 求逆运动学失败,新位置不合法

        // 完成
        for (int i = 0; i < 3; ++i) {
            // 约束增量在+-pi之间
            q[i] = clamp_pi(n_q[i] - q[i]) + q[i];
        }
        return true;
    }

} // namespace ercp
