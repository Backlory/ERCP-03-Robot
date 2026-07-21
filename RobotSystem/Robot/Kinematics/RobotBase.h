/**
 * @file RobotBase.h
 * @author ytom(杨涛)(yangtao1@sia.cn)
 * @brief 机器人基座(平面)运动学
 * @version 0.1
 * @date 2021-03-22
 *
 * @copyright Copyright (c) 2021
 *
 *  OO===(l1)===O===(l2)===E
 *  ^          ^          ^
 *  |          |          |
 * (q1)       (q2)       (q3)
 */

#ifndef __ROBOTBASE__
#define __ROBOTBASE__
namespace ercp {

    class RobotBase {

    public:
        // 连杆1长度[m]
        static const double L1;
        // 连杆2长度[m]
        static const double L2;
        // 虚拟连杆3_边1长度[m]
        static const double L3_1;
        // 虚拟连杆3_边2长度[m]
        double L3_2;
        //初始位姿解算结果（连杆2）
        bool pos_estimate = false;
        double x_init;
        double y_init;
        double th_init;//虚拟连杆3_边2与世界坐标系X轴夹角/rad

        double x_init_new;
        double y_init_new;
        //初始位姿解算结果（虚拟连杆3）
        bool start_telecentric = false;
        double L3_x_init;
        double L3_y_init;
        double L3_th_init;//虚拟连杆3与世界坐标系X夹轴角/rad
    public:
        /**
         * @brief 正向运动学
         *
         * @param q 关节坐标
         * @param x [Out]返回x坐标
         * @param y [Out]返回y坐标
         * @param theta [Out]返回姿态角度，以x轴正方向为0°，逆时针为正
         * @return true
         * @return false
         */
        static bool ForwardKinematics(const double q_in[3], double& x, double& y, double& theta);

        /**
         * @brief 逆向运动学
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
        static bool InverseKinematics_new(const double& x,
            const double& y,
            const double& theta,//弧度制
            double        q_out[3], //计算角度
            const double q_org[3]);//原始角度 

        static bool InverseKinematics(const double& x,
            const double& y,
            const double& theta,
            double        q[3],
            const bool& degree = false,
            const bool& result2 = false);

        /**
         * @brief 对末端沿着力的方向进行拖动
         *
         * @param q [In/Out]关节坐标
         * @param force 作用力(fx,fy,tq)
         * @param velocity 运动速度大小(非负数)[m/s]
         * @param angvel 角速度大小(非负数)[rad/s]
         * @param dt 运动时间(正数)[s]
         * @param senseforce 线性力敏感度(正数),作为fx,fy的最大范围进行归一化,任意负值(默认)表示不进行处理
         * @param sensetorque 扭矩敏感度(正数),作为tq的最大范围进行归一化,任意负值(默认)表示不进行处理
         * @param deadzone 力响应死区(范围+-1),不使用senseforce或sensetorque时无效果,否则比例在对应范围内作为0处理
         * @return true
         * @return false
         */
        static bool DragMoving(double        q[3],
            const double  force[3],
            const double& velocity,
            const double& angvel,
            const double& dt,
            const double& senseforce = -1,
            const double& sensetorque = -1,
            const double& deadzone = 0.2);

         /**
         * @brief 按键移动
         *
         * @param q [In/Out]关节坐标
         * @param key_move 移动方向(fx,fy,tq)
         * @param deadzone
         * 响应死区
         * @return true
         * @return false
         */
        static bool KeyMoving(double q[3], const double key_move[3], const double &deadzone = 0.2);
    
    };
    
} // namespace ercp
#endif // __ROBOTBASE__
