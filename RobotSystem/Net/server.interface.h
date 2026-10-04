#pragma once
#include <vector>
#include <string>
#include "yunsbot_config.h"
#include "RPC/Module.hpp"

namespace server {

namespace robot {

bool init();
bool close();
bool emergency_stop(bool active);

std::string get_robot_name();
std::string get_robot_version();
std::string get_robot_location();

std::string get_time();

bool is_robot_running();
bool is_logging();

bool switch_log();
double force_record();

std::vector<std::string> get_modules();
int get_module_state(std::string);
std::string get_module_step(std::string);
module::FailureInfo get_module_failure(const std::string &type);

std::vector<std::string> get_module_actions();
bool do_module_action(std::string, std::string);


} // namespace robot

namespace settings {

std::string get_setting_config();
std::string get_settings();
bool update_settings(std::string);

} // namespace settings

namespace sensor {

std::vector<int> get_sensors();
std::string get_sensor_name(int id);

} // namespace sensor

namespace gpio {
std::vector<gpio_input_t> get_gpio_inputs();
std::string get_input_name(gpio_input_t id);
std::vector<gpio_output_t> get_gpio_outputs();
std::string get_output_name(gpio_output_t id);
} // namespace gpio

namespace beckhoffator {
bool isOpen();
}
} // namespace server
