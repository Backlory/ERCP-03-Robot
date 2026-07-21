#include <boost/bimap.hpp>
#include <boost/assign.hpp>

#include "server.interface.h"

#include "robot_devices.h"
#include "Device.hpp"
#include "robot_settings.hpp"
#include "robot_config.h"
#include "Robot/YunSBot.h"

////////
#include "RPC/ArmModule.hpp"

using namespace ercp;

namespace server {

    namespace motor {
        extern std::map<motor_t, std::string> motor_names;
    }

    namespace robot {

        bool init() { return YunSBot::GetInstance().base.Start(1); }

        bool start() { return true; }

        bool close() { return YunSBot::GetInstance().base.Stop(); }

        bool interrupt() { return true; }

        bool skip() { return true; }

        //-----------------------------------------------------------------------------

        std::string get_robot_name() { return "YunSBot v2"; }

        std::string get_robot_version() { return "v0.1.0000"; }

        std::string get_robot_location() { return u8"辽宁沈阳"; }

        std::string get_time() { return ilsr::Time::logtime(); }

        bool is_robot_ready() { return true; }

        bool is_robot_running() { return YunSBot::GetInstance().base.IsRobotRunning(); }

        bool is_robot_errored() { return false; }

        bool is_logging() { return YunSBot::GetInstance().base.IsLogging(); }

        bool switch_log() { return YunSBot::GetInstance().base.SwitchLogger(!is_logging()); }

        double force_record() {
            double dForceValue = GetRobot().BeckhoffForce(5);

            ////2-3-2
            //double asex_Pos[21];
            //GetRobot().BeckhoffReadAsexPos(asex_Pos);

            //2-3-3
            double asex_Pos[19];
            GetRobot().BeckhoffReadAsexPos(asex_Pos);

            YunSBot::GetInstance().base.AddFRecord(dForceValue, -asex_Pos[11]);
            return dForceValue;
        }

        //-----------------------------------------------------------------------------

        std::vector<std::string> get_modules()
        {
            static std::vector<std::string> modules{ "arm" };
            return modules;
        }

        int get_module_state(std::string type)
        {
            if (type == "arm") {
                auto &arm = rpc::ArmModule::GetInstance();
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

        std::vector<std::string> get_module_actions()
        {
            static std::vector<std::string> actions{ "suspend", "resume", "clear", "fold", "open",
                "follow", "exit", "auto" };
            return actions;
        }

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
                    if (!yunsbot.base.IsChangeModel()) return false;
                    return arm.GotoState(rpc::arm_state_t::A3_Folded);
                } else if (act == "open") {
                    if (!yunsbot.base.IsChangeModel()) return false;
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

        std::map<std::string, std::function<device::state_t()>> device_mapper{
            { u8"视觉",
                []() {
                    return YunSBot::GetInstance().visual.IsVisionOnline() //
                        ? device::state_t::Online
                        : device::state_t::Offline;
                } },
            { u8"主端",
                []() {
                    return YunSBot::GetInstance().master.IsMasterOnline() //
                        ? device::state_t::Online
                        : device::state_t::Offline;
                } },
        };

        int get_device_state(std::string type)
        {
            if (device_mapper.find(type) == device_mapper.end()) {
                return -1;
            }
            return (int)device_mapper.at(type)();
        }

    } // namespace robot

    //-----------------------------------------------------------------------------

#include <sstream>

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

    namespace motor {

        // clang-format off
    using bimotor = boost::bimaps::bimap<motor_t, std::string>;
    bimotor motor_mapper = boost::assign::list_of<bimotor::relation>
        (motor_t::arm_1, "a1")
        (motor_t::arm_2, "a2")
        (motor_t::arm_3, "a3")
        (motor_t::arm_4, "a4")
        (motor_t::feed_move, "fm")
        (motor_t::feed_clamp, "fc")
        (motor_t::oper_big, "ob")
        (motor_t::oper_small, "os")
        (motor_t::oper_pincer, "op")
        (motor_t::oper_rotate, "or")
        (motor_t::base_1, "b1")
        (motor_t::base_2, "b2")
        (motor_t::base_3, "b3")
        (motor_t::cutter_rot, "cr")
        (motor_t::cutter_feed, "cf")
        (motor_t::cutter_bend, "cc")
        (motor_t::cutter_push, "cp");

    std::map<motor_t, std::string> motor_names{
        {motor_t::arm_1,           u8"臂1"},
        {motor_t::arm_2,           u8"臂2"},
        {motor_t::arm_3,           u8"臂3"},
        {motor_t::arm_4,           u8"臂4"},
        {motor_t::feed_move,       u8"输送"},
        {motor_t::feed_clamp,      u8"夹紧"},
        {motor_t::oper_big,        u8"大拨轮"},
        {motor_t::oper_small,      u8"小拨轮"},
        {motor_t::oper_pincer,     u8"抬钳器"},
        {motor_t::oper_rotate,     u8"镜体旋转"},
        {motor_t::base_1,          u8"基座1"},
        {motor_t::base_2,          u8"基座2"},
        {motor_t::base_3,          u8"基座3"},
        {motor_t::cutter_rot,      u8"切开刀旋转"},
        {motor_t::cutter_feed,     u8"切开刀输送"},
        {motor_t::cutter_bend,     u8"切开刀弯曲"},
        {motor_t::cutter_push,     u8"切开刀送刀"},
    };
        // clang-format on

        motor_t str2mid(std::string id) { return motor_mapper.right.at(id); }

        std::string mid2str(motor_t id) { return motor_mapper.left.at(id); }

        bool valid_str(std::string id)
        {
            return motor_mapper.right.find(id) != motor_mapper.right.end();
        }

        bool valid_mid(motor_t id) { return motor_mapper.left.find(id) != motor_mapper.left.end(); }

    } // namespace motor

    //-----------------------------------------------------------------------------

    namespace sensor {

        double force() { return GetRobot().GetScopeForce(); }

        double torque() { return GetRobot().GetScopeTorque(); }

        double cannula_force() { return GetRobot().GetCannulaForce(); }

        double wire_force() { return GetRobot().GetWireForce(); }

        double operator_force() { return GetRobot().GetHandleForce(); }

        std::map<int, std::string> sensors_name{ //
            { 3, u8"操作器 / 拉压力" }, { 4, u8"切开刀 / 输送力" }, { 5, u8"导丝 / 输送力" },
            { 6, u8"镜体 / 输送力" }, { 7, u8"镜体 / 旋转扭矩" }
        };

        std::map<int, std::function<double()>> sensors_api{ //
            { 3, operator_force }, { 4, cannula_force }, { 5, wire_force }, { 6, force },
            { 7, torque }
        };

        std::vector<int> get_sensors()
        {
            std::vector<int> ids(sensors_name.size());
            std::transform(sensors_name.begin(), sensors_name.end(), ids.begin(),
                [](auto m) { return m.first; });
            return ids;
        }

        std::string get_sensor_name(int id)
        {
            if (sensors_name.find(id) != sensors_name.end()) {
                return sensors_name.at(id);
            }
            return "";
        }

        double get_sensor_value(int id)
        {
            if (sensors_api.find(id) != sensors_api.end()) {
                return sensors_api.at(id)();
            }
            return 0;
        }

    } // namespace sensor

    //-----------------------------------------------------------------------------

    namespace gpio {

        // clang-format off
    static const std::map<gpio_input_t, std::string> InputIOConfigs = {

    };

    static const std::map<gpio_output_t, std::string> OutputIOConfigs = {
       { gas   , u8"打气" },
       { water , u8"喷水" },
       { suct  , u8"吸取" },
    };
        // clang-format on

        std::vector<gpio_input_t> get_gpio_inputs()
        {
            std::vector<gpio_input_t> ids(InputIOConfigs.size());
            std::transform(InputIOConfigs.begin(), InputIOConfigs.end(), ids.begin(),
                [](auto m) { return m.first; });
            return ids;
        }

        std::string get_input_name(gpio_input_t id)
        {
            if (InputIOConfigs.find(id) != InputIOConfigs.end()) {
                return InputIOConfigs.at(id);
            }
            return "";
        }

        //-----------------------------------------------------------------------------

        std::vector<gpio_output_t> get_gpio_outputs()
        {
            std::vector<gpio_output_t> ids(OutputIOConfigs.size());
            std::transform(OutputIOConfigs.begin(), OutputIOConfigs.end(), ids.begin(),
                [](auto m) { return m.first; });
            return ids;
        }

        std::string get_output_name(gpio_output_t id)
        {
            if (OutputIOConfigs.find(id) != OutputIOConfigs.end()) {
                return OutputIOConfigs.at(id);
            }
            return "";
        }

        bool set_output_state(gpio_output_t id, bool on) { return true; }

        uint32_t get_outputs_state() { return true; }

    } // namespace gpio

    //-----------------------------------------------------------------------------

    namespace manuplator {

        bool get_arm_pose(Sophus::Vector6d &pose) { return true; }

        bool get_arm_joints(Sophus::Vector6d &pos) { return true; }

        bool get_arm_velocity(Sophus::Vector6d &vel) { return true; }

        bool get_arm_target(Sophus::Vector6d &vel) { return true; }

        bool is_arm_stopped() { return true; }

        bool is_arm_arrived() { return true; }

        //-----------------------------------------------------------------------------

        bool move_arm_rel(Sophus::Vector6d dpose, bool frame_base) { return true; }

        bool init_arm() { return true; }

        bool stop_arm() { return true; }

        bool is_arm_inited() { return true; }

        bool follow_one_click(double targets, double bigAngle, double smlAngle) { 
            return GetRobot().BeckhoffFollowData_Oneclick(targets, bigAngle, smlAngle);
        }


    } // namespace manuplator

    //-----------------------------------------------------------------------------
    namespace beckhoffator {
        bool isOpen() {
            return GetRobot().IsOpen();
        }

    } // namespace beckhoffator

} // namespace server

