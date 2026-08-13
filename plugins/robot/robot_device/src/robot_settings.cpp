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
std::vector<PropertyBase *> PropertyBase::properties;

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
    std::atomic_int is_loaded = {0};
};

SettingManager::SettingManager()
{
    void make_prop_info();
    make_prop_info();
    LoadSettings();
}

/**
 * @brief 功能：从 config.yaml 读取设置并转换为类型化 Settings 对象。
 * @details 机制：按需跳过已加载配置，解析失败返回 -1 并记录错误，避免错误配置静默回落为默认值。
 */
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
        if (is_loaded < 0) {
            // M5: 解析失败不再静默当成功(否则 basic.master 等回落默认值),
            //     向上返回 -1,由启动流程拒绝进入运行态(见 RobotSystem/main.cpp)。
            ROBOT_ERROR(true, "Failed to parse settings file: " << setfile)
        }
    } catch (std::exception &e) {
        is_loaded = -1;
        ROBOT_ERROR(true, "Failed to load settings file: " << setfile << " (" << e.what() << ")")
    }
    return is_loaded;
}

/**
 * @brief 功能：返回当前设置的 YAML 源数据副本。
 * @details 机制：重新执行解析以补齐默认键；只有解析成功才返回有效节点，否则返回空节点。
 */
YAML::Node SettingManager::GetSource()
{
    YAML::Node config;
    if (Parse(config) > 0) {
        return config;
    }
    return YAML::Node();
}

/**
 * @brief 功能：用新的 YAML 配置更新内存设置并持久化到配置文件。
 * @details 机制：先通过统一 Parse 校验和填充默认值，再仅在解析成功时写回文件。
 */
bool SettingManager::UpdateSource(YAML::Node config)
{
    YAML::Node set;
    is_loaded = Parse(set, config);
    return is_loaded > 0 && SaveSettings();
}

/**
 * @brief 功能：解析 YAML 配置，补齐缺省节点并同步到类型化 Settings 字段。
 * @details 机制：宏只负责声明命名空间/字段访问，实际流程按 basic、device/beckhoff 的固定顺序执行；任一类型转换异常都会清空输出并返回失败。
 */
int SettingManager::Parse(YAML::Node &config, const YAML::Node &input)
{
    // 阶段一：以输入为基线，后续按配置域逐项读取或写入默认值。
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

    // 阶段二：执行基础运行参数和 Beckhoff 连接参数的类型化映射。
    try {
        // clang-format off
            BEGIN_NS("basic")
                CONFIG(Basic.Verbose,   "verbose")
                CONFIG(Basic.Master,    "master")
                CONFIG(Basic.Cloud,     "cloud")
                CONFIG(Basic.Bind,          "bind")
                CONFIG(Basic.CorsOrigin,    "cors_origin")
                CONFIG(Basic.MasterPriority,"master_priority")
            END_NS()

            BEGIN_NS("device")
                BEGIN_SUB_NS("beckhoff")
                    CONFIG(Device.Beckhoff.Addr,    "address")
                    CONFIG(Device.Beckhoff.Port,    "port")
                    CONFIG(Device.Beckhoff.Transport, "transport")
                    CONFIG(Device.Beckhoff.TcpHost,   "tcp_host")
                    CONFIG(Device.Beckhoff.TcpPort,   "tcp_port")
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

/**
 * @brief 功能：将当前类型化设置重新生成 YAML 并保存到 config.yaml。
 * @details 机制：先调用 Parse 生成包含默认值的完整配置，解析成功后覆盖写入文件。
 */
bool SettingManager::SaveSettings()
{
    YAML::Node config;
    if (Parse(config) > 0) {
        std::ofstream(setfile) << config;
        return true;
    }
    return false;
}

ROBOT_API_MEMBER const Settings &GetSettings()
{
    return SettingManager::Get();
}

ROBOT_API_MEMBER std::string GetSettingsPath()
{
    return setfile;
}

ROBOT_API int LoadSettings(bool reload)
{
    return SettingManager::Get().LoadSettings(reload);
}

ROBOT_API void SaveSettings()
{
    SettingManager::Get().SaveSettings();
}

/**
 * @brief 解析 IPv4 或 IPv4:端口形式的配置文本。
 * @details 地址部分始终写回 addr；只有文本包含端口时才更新 port，格式不匹配或端口为空则返回 false。
 */
ROBOT_API bool ParseIPAddress(const std::string &source, std::string &addr, size_t &port)
{
    // 解析 IPv4 或 IPv4:port 配置文本；不带端口时只更新地址，格式错误返回 false。
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

ROBOT_API_MEMBER YAML::Node GetSettingSource()
{
    return SettingManager::Get().GetSource();
}

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

/**
 * @brief 构造配置项的属性元数据树。
 * @details 集中登记命名空间、字段类型、默认值、取值范围和中文说明，供配置查询/编辑接口展示。
 */
void make_prop_info()
{
    // 构造配置元数据树，向 HTTP/UI 提供字段类型、默认值和中文说明；不读取运行时配置文件。

    // clang-format off
    {
        auto &basic = prop_root.emplace_ns("basic");
        basic.emplace("verbose", types::_int, 1)
            .set_comment(u8"选择日志输出详细等级。0表示最少, 2表示最多。")
            .set_tag<types::_int>(std::string(""), std::array<int, 2>{ 0, 2 });
        basic.emplace("master", types::_string, std::string("192.168.1.100"))
            .set_comment(u8"主端IP。");
        basic.emplace("cloud", types::_string, std::string("127.0.0.1"))
            .set_comment(u8"云端IP。");
        basic.emplace("bind", types::_string, std::string("0.0.0.0"))
            .set_comment(u8"7998 HTTP控制面监听网卡地址。0.0.0.0=全网卡。");
        basic.emplace("cors_origin", types::_string, std::string(""))
            .set_comment(u8"7998 CORS允许来源。空=不输出该头。");
        basic.emplace("master_priority", types::_bool, true)
            .set_comment(u8"Master强制优先仲裁开关。自主模式下Master命令<200ms时优先。");
    }

    {
        auto &device = prop_root.emplace_ns("device");
        {
            auto &beckhoff = device.emplace_ns("beckhoff");
            beckhoff.emplace("address", types::_string, std::string("127.0.0.1"))
                .set_comment(u8"倍福控制器地址。");
            beckhoff.emplace("port", types::_int, 851)
                .set_comment(u8"倍福控制器端口。");
            beckhoff.emplace("transport", types::_string, std::string("twincat"))
                .set_comment(u8"ADS传输：twincat或direct。");
            beckhoff.emplace("tcp_host", types::_string, std::string("127.0.0.1"))
                .set_comment(u8"direct模式的ADS/TCP主机。");
            beckhoff.emplace("tcp_port", types::_int, 48898)
                .set_comment(u8"direct模式的ADS/TCP端口。");
        }
    }

    // clang-format on
}
