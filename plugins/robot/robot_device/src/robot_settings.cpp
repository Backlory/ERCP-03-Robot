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
std::vector<PropertyBase*> PropertyBase::properties;

namespace YAML {


} // namespace YAML

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
                CONFIG(Basic.Verbose,   "verbose")
                CONFIG(Basic.Master,    "master")
                CONFIG(Basic.Cloud,     "cloud")
            END_NS()

            BEGIN_NS("device")
                BEGIN_SUB_NS("module")
                    CONFIG(Device.Module.Base,     "use_base")
                    CONFIG(Device.Module.Arm,      "use_arm")
                    CONFIG(Device.Module.Operator, "use_operator")
                    CONFIG(Device.Module.Cannula,  "use_cannula")
                END_SUB_NS()
                BEGIN_SUB_NS("beckhoff")
                    CONFIG(Device.Beckhoff.Addr,    "address")
                    CONFIG(Device.Beckhoff.Port,    "port")
                END_SUB_NS()
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
        basic.emplace("verbose", types::_int, 1)
            .set_comment(u8"选择日志输出详细等级。0表示最少, 2表示最多。")
            .set_tag<types::_int>(std::string(""), std::array<int, 2>{ 0, 2 });
        basic.emplace("master", types::_string, std::string("192.168.1.100"))
            .set_comment(u8"主端IP。");
        basic.emplace("cloud", types::_string, std::string("192.168.1.100"))
            .set_comment(u8"云端IP。");
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

        }
        {
            auto &beckhoff = device.emplace_ns("beckhoff");
            beckhoff.emplace("address", types::_string, std::string("127.0.0.1"))
                .set_comment(u8"倍福控制器地址。");
            beckhoff.emplace("port", types::_int, 851)
                .set_comment(u8"倍福控制器端口。");
        }
    }

    // clang-format off
}
