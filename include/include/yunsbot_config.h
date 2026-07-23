#pragma once

#include <cstddef>
#include <stdint.h>

// This header contains Beckhoff/PLC domain types only. It is not a network ABI.
// Robot UDP V2 is defined and encoded explicitly in protocol/robot_udp_v2.hpp.
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
    BAMS_FOLLOWED = 31,				// 跟随完成
};

enum beckhoff_arm_operation : uint32_t {
    BAO_NONE = 0,
    BAO_FOLD = 1,
    BAO_OPEN = 2,
    BAO_FOLLOW = 3
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
};

