#include <map>
#include "cml_driver.hpp"
#include "yunsbot_config.h"

using namespace device;

namespace ercp {

    extern const double M1_Fold;
    extern const double M2_Fold;
    extern const double M3_Fold;
    extern const double M4_Fold;

    // 关节参数列表
    extern const std::map<motor_t, cml::JointMotorParam> JointConfigs_Auhua = {
       { motor_t::base_0,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 160,
                /* .min_position   = */ 10,
                /* .max_speed      = */ 10.0,
                /* .encoder_offset = */ -42408) },
        { motor_t::base_1,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 0,
                /* .min_position   = */ 0,
                /* .max_speed      = */ 10.0,
                /* .encoder_offset = */ -42408) },
        { motor_t::base_2,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 0,
                /* .min_position   = */ 0,
                /* .max_speed      = */ 10.0,
                /* .encoder_offset = */ 10814) },
        { motor_t::base_3,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 0,
                /* .min_position   = */ 0,
                /* .max_speed      = */ 10.0,
                /* .encoder_offset = */ -35580) },
        { motor_t::feed_move,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 2048.0,
                /* .gear_ratio     = */ 47.0, // 电机减速比47
                /* .max_position   = */ 0,
                /* .min_position   = */ 0,
                /* .max_speed      = */ 90.0) },
        // #define MFc_CTSPDEG (673.8)   //输送器旋转 编码器4096，电机减速比42.3:1，齿轮减速比40:56
        // (0~-M6_NegLimit) #define MFc_CTSPDEG (374.33)
        // 旧：Maxon 400527: 输送器 编码器2048，减速箱47:1    
        // 新：Maxon 283840: 输送器 编码器2048，减速箱270:1   
        { motor_t::feed_clamp,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 2048.0,
                /* .gear_ratio     = */ 270.0 * 56 / 40, // 电机减速比47，齿轮传动比40:56   旧：47.0  新：270.0
                /* .max_position   = */ 0,
                /* .min_position   = */ 0,
                /* .max_speed      = */ 30.0) },

        { motor_t::arm_1,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 60.0,
                /* .min_position   = */ -10.0,
                /* .max_speed      = */ 10.0,
                /* .encoder_offset = */ M1_Fold) },
        { motor_t::arm_2,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 360.0,
                /* .min_position   = */ -360.0,
                /* .max_speed      = */ 15.0,
                /* .encoder_offset = */ M2_Fold,
                /* .max_accel      = */ 200.0,
                /* .max_deacc      = */ 200.0) },
        { motor_t::arm_3,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 360.0,
                /* .min_position   = */ -360.0,
                /* .max_speed      = */ 15.0,
                /* .encoder_offset = */ M3_Fold) },
        { motor_t::oper_rotate,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 65536.0,
                /* .gear_ratio     = */ 101.0,
                /* .max_position   = */ 200.0,
                /* .min_position   = */ -200.0,
                /* .max_speed      = */ 30.0) },


        // #define MFm_CTSPDEG    (591.6)  // 输送器输送 编码器2048，电机减速比104
        // #define MFm_CTSPDEG    (267.4)  // Maxon 400527: 输送器 编码器2048，
        // #define MFmp_CTSPROUND (512.0)  // 被动轮一圈码盘

        //oper_pincer 暂时给新机器人一轴升降使用
        { motor_t::oper_pincer,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 2048.0,
                /* .gear_ratio     = */ 410.0 * 2, // 电机减速比410，齿轮减速比1:2
                /* .max_position   = */ 50.0,
                /* .min_position   = */ 0.0,
                /* .max_speed      = */ 15.0) },
        //#define MOb_CTSPDEG (1972.1)
        { motor_t::oper_big,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 2048.0,
                /* .gear_ratio     = */ 104.0 * 40 / 12, // 电机减速比104，传动带减速比12:40
                /* .max_position   = */ 90.0,   //130.0
                /* .min_position   = */ -135.0,
                /* .max_speed      = */ 40.0) },
        //* .home_offset = */ -317.45
        //#define MOs_CTSPDEG (1451.4)
        { motor_t::oper_small,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 2048.0,
                /* .gear_ratio     = */ 157.0 * 26 / 16, // 电机减速比157，传动带减速比16:26
                /* .max_position   = */ 130.0, // 100
                /* .min_position   = */ -100.0, // 100
                /* .max_speed      = */ 40.0) },
        //* .home_offset = */ -259.5
        //#define MOp_CTSPDEG (4664.8)


        { motor_t::cutter_rot,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 1024.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 5 * 360,
                /* .min_position   = */ -5 * 360,
                /* .max_speed      = */ 180.0) },
        { motor_t::cutter_feed,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 1024.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 0,
                /* .min_position   = */ 0,
                /* .max_speed      = */ 90.0) },
        { motor_t::arm_4,
            cml::JointMotorParam(
                /* .cts_per_round  = */ 131072.0,
                /* .gear_ratio     = */ 1.0,
                /* .max_position   = */ 360.0,
                /* .min_position   = */ -360.0,
                /* .max_speed      = */ 10.0,
                /* .encoder_offset = */ M4_Fold) },
    };

} // namespace ercp
