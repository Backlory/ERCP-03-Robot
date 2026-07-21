#pragma once
#include <vector>
#include <sophus/so3.hpp>
#include "robot_config.h"
#include "yunsbot_config.h"

namespace server {

    namespace robot {

        bool init();
        bool start();
        bool close();
        bool interrupt();
        bool skip();

        std::string get_robot_name();
        std::string get_robot_version();
        std::string get_robot_location();

        std::string get_time();

        bool is_robot_ready();
        bool is_robot_running();
        bool is_robot_errored();
        bool is_logging();
        
        bool switch_log();
        double force_record();

        std::vector<std::string> get_modules();
        int get_module_state(std::string);
        std::string get_module_step(std::string);

        std::vector<std::string> get_module_actions();
        bool do_module_action(std::string, std::string);

        int get_device_state(std::string);

    } // namespace robot

    namespace settings {

        std::string get_setting_config();
        std::string get_settings();
        bool update_settings(std::string);

    } // namespace settings

    namespace motor {
        bool valid_str(std::string id);
        bool valid_mid(motor_t id);
        motor_t str2mid(std::string id);
        std::string mid2str(motor_t id);

    } // namespace motor

    namespace sensor {

        std::vector<int> get_sensors();
        std::string get_sensor_name(int id);
        double get_sensor_value(int id);

    } // namespace sensor

    namespace gpio {

        std::vector<gpio_input_t> get_gpio_inputs();
        std::string get_input_name(gpio_input_t id);

        std::vector<gpio_output_t> get_gpio_outputs();
        std::string get_output_name(gpio_output_t id);
        bool get_output_state(gpio_output_t id);
        bool set_output_state(gpio_output_t id, bool);
        uint32_t get_outputs_state();
        bool set_outputs_state(uint32_t);

    } // namespace gpio

    namespace manuplator {

        bool get_arm_pose(Sophus::Vector6d &pose);
        bool get_arm_joints(Sophus::Vector6d &pos);
        bool get_arm_velocity(Sophus::Vector6d &vel);
        bool get_arm_target(Sophus::Vector6d &vel);

        bool is_arm_stopped();
        bool is_arm_arrived();

        bool move_arm_rel(Sophus::Vector6d dpose, bool frame_base);
        bool move_arm_abs(Sophus::Vector6d pose);

        bool init_arm();
        bool stop_arm();
        bool is_arm_inited();

        bool follow_one_click(double targets, double bigAngle, double smlAngle);

    } // namespace manuplator

    namespace beckhoffator {
        bool isOpen();
    }
} // namespace server
