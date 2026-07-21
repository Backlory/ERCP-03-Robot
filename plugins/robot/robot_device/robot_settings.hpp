#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <Eigen/Dense>
#include <Sophus/se3.hpp>
#include <yaml-cpp/yaml.h>
#include "robot_config.h"
#include "include/settings.hpp"

namespace ercp {

    typedef enum : int {
        Unknown = -1,
        Olympus_H260 = 0,
        Auhua = 1,
    } ScopeType_t;

    struct Settings {
    public:
        struct {
            PropertyReadOnly<bool> Debug = false;
            PropertyReadOnly<int> Verbose = 1;
            PropertyReadOnly<std::string> Master = std::string("127.0.0.1"); //("192.168.1.100");
            PropertyReadOnly<std::string> Cloud = std::string("192.168.1.100"); // 云端
        } Basic;

        struct {
            PropertyReadOnly<bool> Joystick = true;
            PropertyReadOnly<int> NdiChannels = 4;
            PropertyReadOnly<int> ScopeType = (Unknown);
            PropertyReadOnly<std::string> PusherCOM = std::string("\\\\.\\COM1");
            PropertyReadOnly<std::string> StepperCOM = std::string("\\\\.\\COM15");
            PropertyReadOnly<std::string> InjectCOM = std::string("\\\\.\\COM7");
            struct {
                PropertyReadOnly<bool> Base = false;
                PropertyReadOnly<bool> Arm = true;
                PropertyReadOnly<bool> Operator = true;
                PropertyReadOnly<bool> Cannula = false;
                PropertyReadOnly<bool> Injector = true;
            } Module;
            struct {
                PropertyReadOnly<double> Big = 0.0;
                PropertyReadOnly<double> Small = 0.0;
            } Offset;
            struct {
                PropertyReadOnly<double> WheelRadius = 14.10;
                PropertyReadOnly<double> FollowRadius = 0.2371;  //0.320
                PropertyReadOnly<double> FollowLength = 0.2312;
            } Follow;
            struct {
                PropertyReadOnly<bool> Enable = true;
                PropertyReadOnly<bool> UseLog = false;
                PropertyReadOnly<std::string> Addr = std::string("192.168.0.30:502");
                PropertyReadOnly<int> ScopeForceChnl = 6;
                PropertyReadOnly<int> ScopeTorqueChnl = 7;
                PropertyReadOnly<int> CannulaForceChnl = 4;
                PropertyReadOnly<int> WireForceChnl = 5;
                PropertyReadOnly<int> Verbose = 1;
            } ITcp;
            struct {
                PropertyReadOnly<std::string> Addr = std::string("127.0.0.1");
                PropertyReadOnly<int> Port = 851;
            } Beckhoff;
        } Device;

        struct {
            PropertyReadOnly<int> Width = 640;
            PropertyReadOnly<int> Height = 480;
            PropertyReadOnly<Eigen::Vector4i> Crop = {};
            PropertyReadOnly<std::string> Path = std::string("0");
        } Image;

        struct {
            PropertyReadOnly<std::string> URDF_Path = std::string("urdf");
        } Visualization;

        struct {
            PropertyReadOnly<bool> All = false;
            PropertyReadOnly<bool> Main = true;
            PropertyReadOnly<bool> Device = true;
            PropertyReadOnly<bool> Sensor = true;
            PropertyReadOnly<bool> Operation = true;
            PropertyReadOnly<bool> Robot = true;
            PropertyReadOnly<bool> Server = true;
        } Log;

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
