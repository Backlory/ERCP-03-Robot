#include <assert.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <memory>
#include <regex>
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
#include "utils.h"
#include "Settings.hpp"

#include "windows.h"

Settings::Settings() {

    try {
        auto setfile = fs::current_path().string() + "\\scesr.yaml";

        if (!fs::exists(setfile)) {
            MessageBoxW(NULL, L"找不到配置文件 scesr.yaml !", L"错误", NULL);
            exit(0);
        }

        // Load settings from yaml file.
        YAML::Node config = YAML::LoadFile("scesr.yaml");

        Test_        = config["basic"]["test_mode"].as<bool>();
        Joystick_    = config["basic"]["use_joystick"].as<bool>();
        Robot_       = config["basic"]["robot"].as<std::string>();
        Port_        = config["basic"]["port"].as<int>();
        Port2_       = config["basic"]["port2"].as<int>();

        Log.AutoAll    = config["logfile"]["auto_all"].as<bool>();
        Log.MainLog_   = Log.AutoAll || config["logfile"]["mainlog"].as<bool>();
        Log.Motor_     = Log.AutoAll || config["logfile"]["motor"].as<bool>();
        Log.NdiSensor_ = Log.AutoAll || config["logfile"]["ndi_sensor"].as<bool>();
        Log.Operation_ = Log.AutoAll || config["logfile"]["operation"].as<bool>();
        Log.Robot_     = Log.AutoAll || config["logfile"]["robot"].as<bool>();

        Image.Width_  = -1;
        Image.Height_ = -1;
        Image.Crop_   = nullptr;
        Image.Path_   = nullptr;
        if (config["image"]) {
            // TODO: 检查参数
            if (config["image"]["width"]) { Image.Width_ = config["image"]["width"].as<int>(); }
            if (config["image"]["height"]) { Image.Height_ = config["image"]["height"].as<int>(); }
            if (config["image"]["crop"]) {
                auto crop   = config["image"]["crop"].as<std::vector<int>>();
                Image.Crop_ = std::make_shared<cv::Rect>(crop[0], crop[1], crop[2], crop[3]);
            }
            if (config["image"]["path"]) {
                Image.Path_ = std::make_shared<std::string>(config["image"]["path"].as<std::string>());
            }
        }

        if (config["visualization"]) {
            if (config["visualization"]["urdf_path"]) {
                Visualization.URDF_Path_ = config["visualization"]["urdf_path"].as<std::string>();
            }
        }

    } catch (std::exception &e) {
        MessageBoxA(NULL, (std::string("加载配置文件失败.\n") + e.what()).c_str(), "错误", NULL);
        exit(0);
    }
};
