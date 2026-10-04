#include <algorithm>
#include <functional>
#include <map>
#include <sstream>

#include "server.interface.h"

#include "robot_devices.h"
#include "robot_settings.hpp"
#include "robot_config.h"
#include "Robot/YunSBot.h"

////////
#include "RPC/ArmModule.hpp"

using namespace ercp;

namespace server {

namespace robot {

bool init()
{
    return YunSBot::GetInstance().base.Start(1);
}

bool close()
{
    return YunSBot::GetInstance().base.Stop();
}

bool emergency_stop(bool active)
{
    return YunSBot::GetInstance().base.SetRpcEmergencyStop(active);
}

//-----------------------------------------------------------------------------

std::string get_robot_name()
{
    return "YunSBot v2";
}

std::string get_robot_version()
{
    return "v0.1.0000";
}

std::string get_robot_location()
{
    return u8"辽宁沈阳";
}

std::string get_time()
{
    return ilsr::Time::logtime();
}

bool is_robot_running()
{
    return YunSBot::GetInstance().base.IsRobotRunning();
}

bool is_logging()
{
    return YunSBot::GetInstance().base.IsLogging();
}

bool switch_log()
{
    return YunSBot::GetInstance().base.SwitchLogger(!is_logging());
}

double force_record()
{
    double dForceValue = GetRobot().BeckhoffForce(5);

    double asex_Pos[19];
    GetRobot().BeckhoffReadAsexPos(asex_Pos);

    YunSBot::GetInstance().base.AddFRecord(dForceValue, -asex_Pos[11]);
    return dForceValue;
}

//-----------------------------------------------------------------------------

std::vector<std::string> get_modules()
{
    static std::vector<std::string> modules{"arm"};
    return modules;
}

int get_module_state(std::string type)
{
    if (type == "arm") {
        auto &arm = rpc::ArmModule::GetInstance();
        if (arm.GetModuleState() == module::module_state::S_Error) return 1;
        if (arm.IsBusy()) {
            return 3;
        }
        return (int)arm.GetModuleState();
    }
    return -1;
}

std::string get_module_step(std::string type)
{
    if (type == "arm") {
        auto &arm = rpc::ArmModule::GetInstance();
        return rpc::GetProcessName(arm.get_current_state());
    }
    return "Unknow";
}

module::FailureInfo get_module_failure(const std::string &type)
{
    if (type == "arm") return rpc::ArmModule::GetInstance().GetFailure();
    throw std::invalid_argument("Unknown module: " + type);
}

std::vector<std::string> get_module_actions()
{
    static std::vector<std::string>
        actions{"suspend", "resume", "clear", "fold", "open", "follow", "exit", "auto"};
    return actions;
}

/**
 * @brief 功能：把 HTTP module/action 请求映射为机械臂模块的状态转移或自动模式切换。
 * @details 机制：先按模块类型选择 ArmModule，再按 action 分派到暂停、恢复、救援、目标状态和跟随控制接口。
 */
bool do_module_action(std::string type, std::string act)
{
    if (type == "arm") {
        auto &yunsbot = ercp::YunSBot::GetInstance();
        auto &arm = rpc::ArmModule::GetInstance();
        if (act == "suspend") {
            return arm.Pause<rpc::ArmModule>();
        } else if (act == "resume") {
            return arm.Resume<rpc::ArmModule>();
        } else if (act == "clear") {
            return arm.Rescue<rpc::ArmModule>(true);
        } else if (act == "fold") {
            return arm.GotoState(rpc::arm_state_t::A3_Folded);
        } else if (act == "open") {
            return arm.GotoState(rpc::arm_state_t::A4_Opened);
        } else if (act == "follow") {
            return arm.StartFollow();
        } else if (act == "exit") {
            return arm.StopFollow();
        } else if (act == "auto") {
            return yunsbot.base.SwitchAutoMode(!yunsbot.base.IsAutoMode());
        }
        return false;
    }
    return false;
}

//-----------------------------------------------------------------------------

} // namespace robot

//-----------------------------------------------------------------------------

namespace settings {

std::string get_setting_config()
{
    auto node = GetSettingConfig();
    std::stringstream s;
    s << node;
    return s.str();
}

std::string get_settings()
{
    auto node = GetSettingSource();
    std::stringstream s;
    s << node;
    return s.str();
}

bool update_settings(std::string config)
{
    auto node = YAML::Load(config);
    return UpdateSettingSource(node);
}

} // namespace settings

//-----------------------------------------------------------------------------

namespace sensor {

std::map<int, std::string> sensors_name{//
                                        {3, u8"操作器 / 拉压力"},
                                        {4, u8"切开刀 / 输送力"},
                                        {5, u8"导丝 / 输送力"},
                                        {6, u8"镜体 / 输送力"},
                                        {7, u8"镜体 / 旋转扭矩"}};

std::vector<int> get_sensors()
{
    std::vector<int> ids(sensors_name.size());
    std::transform(sensors_name.begin(), sensors_name.end(), ids.begin(), [](auto m) {
        return m.first;
    });
    return ids;
}

std::string get_sensor_name(int id)
{
    if (sensors_name.find(id) != sensors_name.end()) {
        return sensors_name.at(id);
    }
    return "";
}

} // namespace sensor

//-----------------------------------------------------------------------------

namespace gpio {

static const std::map<gpio_input_t, std::string> InputIOConfigs = {};
static const std::map<gpio_output_t, std::string> OutputIOConfigs = {
    {gas, u8"打气"}, {water, u8"喷水"}, {suct, u8"吸取"}};

std::vector<gpio_input_t> get_gpio_inputs()
{
    std::vector<gpio_input_t> ids;
    for (const auto &entry : InputIOConfigs)
        ids.push_back(entry.first);
    return ids;
}

std::string get_input_name(gpio_input_t id)
{
    const auto entry = InputIOConfigs.find(id);
    return entry != InputIOConfigs.end() ? entry->second : "";
}

std::vector<gpio_output_t> get_gpio_outputs()
{
    std::vector<gpio_output_t> ids;
    for (const auto &entry : OutputIOConfigs)
        ids.push_back(entry.first);
    return ids;
}

std::string get_output_name(gpio_output_t id)
{
    const auto entry = OutputIOConfigs.find(id);
    return entry != OutputIOConfigs.end() ? entry->second : "";
}

} // namespace gpio

namespace beckhoffator {
bool isOpen()
{
    return GetRobot().IsOpen();
}

} // namespace beckhoffator

} // namespace server
