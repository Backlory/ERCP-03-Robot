#pragma once
#include <stdint.h>
#include <yaml-cpp/yaml.h>
#include "robot_config.h"
#include "include/settings.hpp"

namespace ercp {


    struct Settings {
    public:
        struct {
            PropertyReadOnly<int> Verbose = 1;
            PropertyReadOnly<std::string> Master = std::string("127.0.0.1"); //("192.168.1.100");
            PropertyReadOnly<std::string> Cloud = std::string("127.0.0.1");
            // 7998 HTTP 控制面监听网卡地址。默认 0.0.0.0 保持现有全网卡可达性,
            // 可收窄到指定内网网卡。空或 0.0.0.0 时按原全网卡行为绑定。
            PropertyReadOnly<std::string> Bind = std::string("0.0.0.0");
            // 7998 CORS 允许来源。默认空 = 不输出 Access-Control-Allow-Origin 头
            // (Master 用 .NET WebRequest 非浏览器,CORS 对其无影响)。
            PropertyReadOnly<std::string> CorsOrigin = std::string("");
            // Master 强制优先仲裁开关(见 ControlRunnable2)。默认启用。
            PropertyReadOnly<bool> MasterPriority = true;
        } Basic;

        struct {
            struct {
                PropertyReadOnly<std::string> Addr = std::string("127.0.0.1");
                PropertyReadOnly<int> Port = 851;
                PropertyReadOnly<std::string> Transport = std::string("twincat");
                PropertyReadOnly<std::string> TcpHost = std::string("127.0.0.1");
                PropertyReadOnly<int> TcpPort = 48898;
            } Beckhoff;
        } Device;




    protected:
        Settings() = default;
        Settings(const Settings &) = delete;
    };

#pragma region api
    ROBOT_API bool ParseIPAddress(const std::string &source, std::string &addr, size_t &port);

    ROBOT_API_MEMBER const Settings &GetSettings();
    ROBOT_API_MEMBER std::string GetSettingsPath();
    ROBOT_API int LoadSettings(bool reload = true);
    ROBOT_API void SaveSettings();

    ROBOT_API_MEMBER YAML::Node GetSettingSource();
    ROBOT_API bool UpdateSettingSource(YAML::Node);

    ROBOT_API_MEMBER YAML::Node GetSettingConfig();
#pragma endregion

} // namespace ercp
