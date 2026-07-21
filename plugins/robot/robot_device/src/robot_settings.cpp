#include <fstream>
#include <memory>
#include <regex>

#if __cplusplus >= 201703L || _MSVC_LANG >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#elif __cplusplus >= 201402L || _MSVC_LANG >= 201402L
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

#include "include/prop.hpp"
#include "robot_settings.hpp"

namespace YAML {

    template <>
    YAML::Node &YAML::Node::operator=(const Eigen::Vector4i &vec)
    {
        std::vector<int> val;
        for (int i = 0; i < vec.rows(); ++i) {
            val.push_back(vec(i));
        }
        return (*this) = val;
    }

} // namespace YAML

// 统一访问接口
std::vector<PropertyBase *> PropertyBase::properties;

// 全局设置
using namespace prop;
prop_namespace prop_root;

namespace ercp {

    static const auto setfile = fs::current_path().string() + "\\config.yaml";

    class SettingManager : public Settings {
    public:
        static SettingManager &Get()
        {
            static SettingManager _set;
            return _set;
        }

        int LoadSettings(bool reload = true);
        bool SaveSettings();
        YAML::Node GetSource();
        bool UpdateSource(YAML::Node);

    protected:
        SettingManager();
        SettingManager(const SettingManager &) = delete;

    private:
        int Parse(YAML::Node &ret, const YAML::Node &node = YAML::Node());

    public:
        /// <summary>
        /// 0 = no file, -1 = load failed, 1 = load succeed
        /// </summary>
        std::atomic_int is_loaded = { 0 };
    };

    SettingManager::SettingManager()
    {
        void make_prop_info();
        make_prop_info();
        LoadSettings();
    }

    int SettingManager::LoadSettings(bool reload)
    {
        if (!reload && is_loaded > 0) {
            return is_loaded;
        }
        try {
            if (!fs::exists(setfile)) {
                is_loaded = 0;
                return is_loaded;
            }
            // Load settings from yaml file.
            YAML::Node set;
            is_loaded = Parse(set, YAML::LoadFile(setfile));

        } catch (std::exception &e) {
            is_loaded = -1;
        }
        is_loaded = 1;
        return is_loaded;
    }

    YAML::Node SettingManager::GetSource()
    {
        YAML::Node config;
        if (Parse(config) > 0) {
            return config;
        }
        return YAML::Node();
    }

    bool SettingManager::UpdateSource(YAML::Node config)
    {
        YAML::Node set;
        is_loaded = Parse(set, config);
        return is_loaded > 0 && SaveSettings();
    }

    int SettingManager::Parse(YAML::Node &config, const YAML::Node &input)
    {
        config = input;
        //---------------------------------------------------------------------
#define BEGIN_NS(ns)                                                                               \
    {                                                                                              \
        const std::string _ns = ns;                                                                \
        if (!config[_ns])                                                                          \
            config[_ns] = YAML::Node();                                                            \
        auto cfg = config[_ns];
        //---------------------------------------------------------------------
#define END_NS()                                                                                   \
    config[_ns] = cfg;                                                                             \
    }
        //---------------------------------------------------------------------
#define BEGIN_SUB_NS(ns2)                                                                          \
    {                                                                                              \
        const std::string _ns2 = ns2;                                                              \
        if (!config[_ns][_ns2])                                                                    \
            config[_ns][_ns2] = YAML::Node();                                                      \
        auto cfg = config[_ns][_ns2];
        //---------------------------------------------------------------------
#define END_SUB_NS()                                                                               \
    config[_ns][_ns2] = cfg;                                                                       \
    }
        //---------------------------------------------------------------------
#define CONFIG(v, key)                                                                             \
    {                                                                                              \
        if (cfg[key]) {                                                                            \
            v = cfg[key].as<decltype(v)::value_type>();                                            \
        } else {                                                                                   \
            cfg[key] = v();                                                                        \
        }                                                                                          \
    }
//---------------------------------------------------------------------
#define CONFIG_ARRAY(v, key, t, N)                                                                 \
    {                                                                                              \
        if (cfg[key]) {                                                                            \
            std::vector<t> tmp;                                                                    \
            tmp = cfg[key].as<std::vector<t>>();                                                   \
            if (tmp.size() != N) {                                                                 \
                cfg[key] = v();                                                                    \
            } else {                                                                               \
                v = Eigen::Map<Eigen::Matrix<t, N, 1>>(tmp.data(), N, 1);                          \
            }                                                                                      \
        } else {                                                                                   \
            cfg[key] = v();                                                                        \
        }                                                                                          \
    }
        //---------------------------------------------------------------------

        try {
            // clang-format off
            BEGIN_NS("basic")
                CONFIG(Basic.Debug,     "debug")
                CONFIG(Basic.Verbose,   "verbose")
                CONFIG(Basic.Master,    "master")
            END_NS()

            BEGIN_NS("device")
                CONFIG(Device.Joystick,     "use_joystic")
                CONFIG(Device.NdiChannels,  "ndi_channels")
                CONFIG(Device.ScopeType,    "scope_type")
                CONFIG(Device.PusherCOM,    "pusher_port")
                CONFIG(Device.StepperCOM,   "stepper_port")
                BEGIN_SUB_NS("offset")
                    CONFIG(Device.Offset.Big,   "big")
                    CONFIG(Device.Offset.Small, "small")
                END_SUB_NS()
                BEGIN_SUB_NS("follow")
                    CONFIG(Device.Follow.WheelRadius, "whl_radius")
                    CONFIG(Device.Follow.FollowRadius, "fol_radius")
                    CONFIG(Device.Follow.FollowLength, "fol_length")
                END_SUB_NS()
                BEGIN_SUB_NS("module")
                    CONFIG(Device.Module.Base,     "use_base")
                    CONFIG(Device.Module.Arm,      "use_arm")
                    CONFIG(Device.Module.Operator, "use_operator")
                    CONFIG(Device.Module.Cannula,  "use_cannula")
                    CONFIG(Device.Module.Injector,  "use_jnject")
                END_SUB_NS()
                BEGIN_SUB_NS("itcp")
                    CONFIG(Device.ITcp.Enable,     "enable")
                    CONFIG(Device.ITcp.UseLog,     "use_log")
                    CONFIG(Device.ITcp.Addr,       "address")
                    CONFIG(Device.ITcp.ScopeForceChnl,   "scope_force")
                    CONFIG(Device.ITcp.ScopeTorqueChnl,  "scope_torque")
                    CONFIG(Device.ITcp.CannulaForceChnl, "cannula_force")
                    CONFIG(Device.ITcp.WireForceChnl,    "wire_force")
                    CONFIG(Device.ITcp.Verbose,    "verbose")
                END_SUB_NS()
                BEGIN_SUB_NS("beckhoff")
                    CONFIG(Device.Beckhoff.Addr,    "address")
                    CONFIG(Device.Beckhoff.Port,    "port")
                END_SUB_NS()
            END_NS()

            BEGIN_NS("image")
                CONFIG(Image.Width,      "witdh")
                CONFIG(Image.Height,     "height")
                CONFIG(Image.Path,       "path")
                CONFIG_ARRAY(Image.Crop, "crop", int, 4)
            END_NS()

            BEGIN_NS("visualization")
                CONFIG(Visualization.URDF_Path,  "urdf_path")
            END_NS()

            BEGIN_NS("log")
                CONFIG(Log.All,         "all")
                CONFIG(Log.Main,        "use_main")
                CONFIG(Log.Device,      "use_device")
                CONFIG(Log.Sensor,      "use_sensor")
                CONFIG(Log.Server,      "use_server")
                CONFIG(Log.Operation,   "use_operation")
                CONFIG(Log.Robot,       "use_robot")
            END_NS()
            // clang-format on

        } catch (std::exception &e) {
            config.reset();
            return -1;
        }
#undef BEGIN_NS
#undef END_NS
#undef CONFIG
#undef CONFIG_ARRAY
        return 1;
    }

    bool SettingManager::SaveSettings()
    {
        YAML::Node config;
        if (Parse(config) > 0) {
            std::ofstream(setfile) << config;
            return true;
        }
        return false;
    }

    ROBOT_API_MEMBER const Settings &GetSettings() { return SettingManager::Get(); }

    ROBOT_API int LoadSettings(bool reload) { return SettingManager::Get().LoadSettings(reload); }

    ROBOT_API void SaveSettings() { SettingManager::Get().SaveSettings(); }

    ROBOT_API bool ParseIPAddress(const std::string &source, std::string &addr, size_t &port)
    {
        static const std::regex ip_regex("^((?:[0-9]{1,3}\\.){3}[0-9]{1,3})(?::([0-9]+))?$");
        std::smatch base_match;
        if (std::regex_match(source, base_match, ip_regex)) {
            if (base_match.size() >= 2)
                addr = base_match[1].str();
            if (base_match.size() >= 3 && base_match[2].length())
                port = std::stoi(base_match[2].str());
            return true;
        }
        return false;
    }

    ROBOT_API_MEMBER YAML::Node GetSettingSource() { return SettingManager::Get().GetSource(); }

    ROBOT_API bool UpdateSettingSource(YAML::Node config)
    {
        return SettingManager::Get().UpdateSource(config);
    }

    ROBOT_API_MEMBER YAML::Node GetSettingConfig()
    {
        return prop::prop_namespace::generate(prop_root);
    }

} // namespace ercp

//-----------------------------------------------------------------------------

void make_prop_info()
{

    // clang-format off
    {
        auto &basic = prop_root.emplace_ns("basic");
        basic.emplace("debug", types::_bool, false)
            .set_comment(u8"是否是调试模式。");
        basic.emplace("verbose", types::_int, 1)
            .set_comment(u8"选择日志输出详细等级。0表示最少, 2表示最多。")
            .set_tag<types::_int>(std::string(""), std::array<int, 2>{ 0, 2 });
        basic.emplace("master", types::_string, std::string("192.168.1.100"))
            .set_comment(u8"主端IP。");
    }

    {
        auto &device = prop_root.emplace_ns("device");
        {
            auto &module = device.emplace_ns("module");
            module.emplace("use_base", types::_bool, false)
                .set_comment(u8"是否启用基座模块。");
            module.emplace("use_arm", types::_bool, true)
                .set_comment(u8"是否启用机械臂模块。");
            module.emplace("use_operator", types::_bool, true)
                .set_comment(u8"是否启用操作器模块。");
            module.emplace("use_cannula", types::_bool, false)
                .set_comment(u8"是否启用插管模块。");

            auto& offset = device.emplace_ns("offset");
            offset.emplace("big", types::_float, 0.0)
                .set_comment(u8"大拨轮偏移角度。");
            offset.emplace("small", types::_float, 0.0)
                .set_comment(u8"小拨轮偏移角度。");

            auto& follow = device.emplace_ns("follow");
            follow.emplace("whl_radius", types::_float, 19.65)
                .set_comment(u8"输送轮名义半径。");
            follow.emplace("fol_radius", types::_float, 0.320)
                .set_comment(u8"输送轮跟随半径。");
            follow.emplace("fol_length", types::_float, 0.150)
                .set_comment(u8"输送轮跟随直线段长度。");

            auto &itcp = device.emplace_ns("itcp");
            itcp.emplace("enable", types::_bool, true)
                .set_comment(u8"是否启用ITcp力感采集模块");
            itcp.emplace("use_log", types::_bool, false)
                .set_comment(u8"是否启用ITcp原始数据记录。");
            itcp.emplace("address", types::_string, std::string("192.168.0.30:502"))
                .set_comment(u8"ITcp模块TCP地址和端口。")
                .set_tag<types::_string>(std::string("ipaddr_port"));
            itcp.emplace("scope_force", types::_int, 6).set_comment(u8"力传感器采集通道。");
            itcp.emplace("scope_torque", types::_int, 7)
                .set_comment(u8"扭矩传感器采集通道。");
            itcp.emplace("cannula_force", types::_int, 4).set_comment(u8"切开刀力感采集通道。");
            itcp.emplace("wire_force", types::_int, 5)
                .set_comment(u8"导丝力感采集通道。");
            itcp.emplace("verbose", types::_int, 1)
                .set_comment(u8"选择日志输出详细等级。参见 \"basic/verbose\"。")
                .set_tag<types::_int>(std::string(""), std::array<int, 2>{ 0, 2 });
            

            device.emplace("use_joystic", types::_bool, true)
                .set_comment(u8"是否启用手柄操作功能(主从模式)。");
            device.emplace("ndi_channels", types::_int, 4)
                .set_comment(u8"NDI电磁传感器通道数量。")
                .set_tag<types::_int>(std::string(u8"个"), std::array<int, 2>{ 1, 8 });
            device.emplace("scope_type", types::_enum, std::string("Olympus"))
                .set_comment(u8"内镜类型。")
                .set_tag<types::_enum>("", std::vector<std::string>{ "Olympus", "Auhua" });
            device.emplace("pusher_port", types::_enum, std::string("\\\\.\\COM1"))
                .set_comment(u8"导管操作器输送电机端口(串口)。")
                .set_tag<types::_enum>(std::string("serial"), std::vector<std::string>{});
            device.emplace("stepper_port", types::_enum, std::string("\\\\.\\COM15"))
                .set_comment(u8"导管牵拉电机端口(串口)。")
                .set_tag<types::_enum>(std::string("serial"), std::vector<std::string>{});
        }
    }

    {
        auto &image = prop_root.emplace_ns("image");
        image.emplace("witdh", types::_int, 640)
            .set_comment(u8"图像输入宽度。");
        image.emplace("height", types::_int, 480)
            .set_comment(u8"图像输入高度。");
        image.emplace("path", types::_string, std::string("0"))
            .set_comment(u8"图像路径，可以是相机计数编号(如:0)或者设备名称(如:UsbCamera)。");
        image.emplace("crop", types::_vector, std::vector<int>{})
            .set_comment(u8"输入图像剪切，(x1,y1,x2,y2)。")
            .set_tag<types::_vector>(std::string(u8"像素"), 4, std::string("int"));
    }

    {
        auto &log = prop_root.emplace_ns("log");
        log.emplace("all", types::_bool, false)
            .set_comment(u8"是否打开全部日志。");
        log.emplace("use_main", types::_bool, true)
            .set_comment(u8"是否启用主界面日志。");
        log.emplace("use_server", types::_bool, true)
            .set_comment(u8"是否启用网络服务日志。");
        log.emplace("use_device", types::_bool, true)
            .set_comment(u8"是否启用设备日志。");
        log.emplace("use_sensor", types::_bool, true)
            .set_comment(u8"是否启用传感器数据记录。");
        log.emplace("use_operation", types::_bool, true)
            .set_comment(u8"是否启用操作行为日志。");
        log.emplace("use_robot", types::_bool, true)
            .set_comment(u8"是否启用机器人日志。");
    }

    {
        auto &vis = prop_root.emplace_ns("visualization");
        vis.emplace("urdf_path", types::_string, std::string("urdf"))
            .set_comment(u8"机器人URDF模型文件路径。");
    }
    // clang-format off
}
