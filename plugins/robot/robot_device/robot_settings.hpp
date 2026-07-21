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
            PropertyReadOnly<std::string> Cloud = std::string("192.168.1.100"); // 云端
        } Basic;

        struct {
            struct {
                PropertyReadOnly<bool> Base = false;
                PropertyReadOnly<bool> Arm = true;
                PropertyReadOnly<bool> Operator = true;
                PropertyReadOnly<bool> Cannula = false;
            } Module;
            struct {
                PropertyReadOnly<std::string> Addr = std::string("127.0.0.1");
                PropertyReadOnly<int> Port = 851;
            } Beckhoff;
        } Device;




    protected:
        Settings() = default;
        Settings(const Settings &) = delete;
    };

#pragma region api
    ROBOT_API bool ParseIPAddress(const std::string &source, std::string &addr, size_t &port);

    ROBOT_API_MEMBER const Settings &GetSettings();
    ROBOT_API int LoadSettings(bool reload = true);
    ROBOT_API void SaveSettings();

    ROBOT_API_MEMBER YAML::Node GetSettingSource();
    ROBOT_API bool UpdateSettingSource(YAML::Node);

    ROBOT_API_MEMBER YAML::Node GetSettingConfig();
#pragma endregion

} // namespace ercp
