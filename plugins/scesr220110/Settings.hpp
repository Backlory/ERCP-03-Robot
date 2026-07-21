#pragma once

#include <filesystem>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

#define PROPERTY(T, x)   \
protected:               \
    T x##_;              \
                         \
public:                  \
    T const &x() const { \
        return x##_;     \
    }

class Settings {
public:
    static const Settings &Get() {
        static Settings _set;
        return _set;
    }

    typedef enum {
        Unknown,
        Olympus_H260,
        Auhua,
    } EndoscopeType_t;

public:
    PROPERTY(bool, Test)
    PROPERTY(bool, Joystick)
    PROPERTY(std::string, Robot)
    PROPERTY(int, Port)
    PROPERTY(int, Port2)

    struct {
        friend class Settings;
        PROPERTY(bool, MainLog)
        PROPERTY(bool, Motor)
        PROPERTY(bool, NdiSensor)
        PROPERTY(bool, Operation)
        PROPERTY(bool, Robot)
    protected:
        bool AutoAll;
    } Log;

    struct {
        friend class Settings;
        PROPERTY(int, Width)
        PROPERTY(int, Height)
        PROPERTY(std::shared_ptr<cv::Rect>, Crop)
        PROPERTY(std::shared_ptr<std::string>, Path)
    } Image;

    struct {
        friend class Settings;
        PROPERTY(std::string, URDF_Path)
    } Visualization;

private:
    Settings();

    Settings(const Settings &) = delete;
};

#undef MEMBER
