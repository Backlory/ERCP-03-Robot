#pragma once

#include <stdint.h>

// clang-format off
enum class motor_t : int {
    // 臂展电机
    arm_1 = (6),
    arm_2 = (7),
    arm_3 = (8),
    // 输送部电机
    feed_move = (4),
    feed_clamp = (5),
    // 内镜操作部电机
    oper_rotate = (9),
    oper_pincer = (10),
    oper_big = (11),
    oper_small = (12),
    // 基座电机
    base_0 = (0),
    base_1 = (1),
    base_2 = (2),
    base_3 = (3),
    // 切开刀操作台电机
    cutter_rot = (14),
    cutter_feed = (15),
    // 其他类型
    cutter_bend = (128),
    cutter_push = (129),
    // 其他类型
    arm_4 = (130),
    copley_gpio = oper_pincer,
};

enum beckhoff_arm_move_state : uint32_t {
    BAMS_START = 0,					// 初始状态
    BAMS_FOLDING = 10,				// 折叠中
    BAMS_FOLDED = 11,				// 折叠完成
    BAMS_OPENING = 20,				// 展开中
    BAMS_OPENED = 21,				// 展开完成
    BAMS_FOLLOWING = 30,			// 跟随中
    BAMS_FOLLOWED = 31,				// 跟谁完成
    BAMS_FOLLOWING_ONE = 40,		// 一键跟随中
    BAMS_FOLLOWED_ONE = 41,			// 一键跟谁完成
};

enum beckhoff_arm_operation : uint32_t {
    BAO_NONE = 0,
    BAO_FOLD = 1,
    BAO_OPEN = 2,
    BAO_FOLLOW = 3,
    BAO_FOLLOW_ONE = 4
};


typedef enum gpio_input_t : uint32_t {

};

typedef enum gpio_output_t : uint32_t {
    valve   = (0b00000001),
    gas     = (0b00001000),
    water   = (0b00000100),
    suct    = (0b00000010),
};
// clang-format on

struct control_cmd {
    double time = -1;
    bool debug = false;
    bool switch_water = false;
    bool switch_gas = false;
    bool switch_suct = false;
    double vel_move = 0;
    double vel_bend_lr = 0;
    double vel_bend_ud = 0;
    double vel_rotate = 0;
    double vel_pincer = 0;
    double follow_comp_botton = 0; //按键跟随补偿
//    double oneclick_follow_target = 0; // 目标跟随位置
    bool home_bend_lr = false;
    bool home_bend_ud = false;
    bool home_rotate = false;
    bool home_cannula_rotate = false;
//    bool oneclick_follow_flag = false; // 一键跟随开启/关闭

    struct {
        bool use_base = false;

        double vel_z = 0;
        double vel_x = 0;
        double vel_y = 0;
        double vel_r = 0;
    } base;

    bool use_cannula = false;
    struct {
        double vel_cutter_move = 0; // 切开刀输送
        double vel_cutter_bend = 0; // 拉弓
        double vel_wire_feed = 0; // 导丝输送
        double vel_cutter_rotate = 0; // 切开刀摆转
    } cannula;

public:
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar &time;
        ar &debug;
        ar &switch_water;
        ar &switch_gas;
        ar &switch_suct;
        ar &vel_move;
        ar &vel_bend_lr;
        ar &vel_bend_ud;
        ar &vel_rotate;
        ar &vel_pincer;
        ar &follow_comp_botton; 
//        ar &oneclick_follow_target;
        ar &home_bend_lr;
        ar &home_bend_ud;
        ar &home_rotate;
        ar &home_cannula_rotate;
//        ar &oneclick_follow_flag;

        ar &base.use_base;
        ar &base.vel_z;
        ar &base.vel_x;
        ar &base.vel_y;
        ar &base.vel_r;

        ar &use_cannula;
        ar &cannula.vel_cutter_move;
        ar &cannula.vel_cutter_bend;
        ar &cannula.vel_wire_feed;
        ar &cannula.vel_cutter_rotate;
    }
};

struct beckhoff_follow_cmd {

    double follow_comp_botton = 0;

    double vel_move = 0;
    double vel_rotate = 0;
    double vel_bend_lr = 0;
    double vel_bend_ud = 0;
    double vel_pincer = 0;

    double vel_cutter_feed = 0;
    double vel_cutter_rot = 0;
    double vel_cutter_bend = 0;
    double vel_wire_feed = 0;

    bool home_rotate = false;
    bool home_bend_lr = false;
    bool home_bend_ud = false;

    bool switch_water = false;
    bool switch_gas = false;
    bool switch_suct = false;
}; // Robot 2-3-1

//struct beckhoff_follow_cmd {
//
//    double follow_comp_botton = 0;
//
//    double vel_move = 0;
//    double vel_rotate = 0;
//    double vel_pincer = 0;
//    double vel_bend_lr = 0;
//    double vel_bend_ud = 0;
//
//    double vel_cutter_rot = 0;
//    double vel_cutter_bend = 0;
//    double vel_wire_feed = 0;
//    double vel_cutter_feed = 0;
//
//    bool home_rotate = false;
//    bool home_bend_lr = false;
//    bool home_bend_ud = false;
//
//    bool switch_water = false;
//    bool switch_gas = false;
//    bool switch_suct = false;
//}; // Robot 2-2

struct robot_status {
    double time = -1;
    float pos_move = 0;
    float pos_bend_lr = 0;
    float pos_bend_ud = 0;
    float pos_rotate = 0;
    float pos_pincer = 0;
    float vel_move = 0;
    float vel_bend_lr = 0;
    float vel_bend_ud = 0;
    float vel_rotate = 0;
    float vel_pincer = 0;

    bool switch_water = false;
    bool switch_gas = false;
    bool switch_suct = false;

    bool robot_starting = false;
    bool robot_stopping = false;
    bool robot_running = false;
    bool robot_folded = false;
    bool robot_opened = false;
    bool robot_following = false;

    float follow_length = 0;

    // Reserved
    float _reserved[64] = { 0 };
    bool _reserved2[32] = { false };
};

struct master_cmd_ex {
    //// 附加主端操作指令
    // bool debug = false;
    // bool clamp = false;
    // bool arm_open = false;
    // bool arm_fold = false;
    // bool feed_start = false;

    // bool cutter_mode = false;
    // bool cutter_vision = false;
    // bool cutter_auto = false;

    // bool base = false;
    // double base_X = 0;
    // double base_Y = 0;
    // double base_R = 0;

    // bool debug_add_length = false;
    // bool debug_sub_length = false;
};
