#pragma once
#ifndef _VIDEOLIST_H_
#define _VIDEOLIST_H_

#include <vector>
#include <string>

struct FrameInfo {
    int width;
    int height;
    float fps;
};

struct FormatInfo {
    std::string vcodec;
    int pixel_format = -1;
    int compress = -1;
    FrameInfo min;
    FrameInfo max;
};

enum class DeviceType {
    VideoInput,
    AudioInput,
    AudioRenderer,
};

#if defined(WINDOWS) || defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
#define VIDEOLIST_WINDOWS
struct DeviceInfo {
    DeviceType DeviceType;
    std::wstring DeviceName;
    std::wstring DevicePath;
    std::wstring MonikerName;
};
#else
#define VIDEOLIST_LINUX
struct DeviceInfo {
    std::string DeviceName;
    std::string DevicePath;
    std::string MonikerName;
};
#endif

bool GetAudioVideoInputDevices(std::vector<DeviceInfo> &devices, DeviceType type);

bool GetCameraFormats(std::vector<std::vector<FormatInfo>> &formats, const DeviceInfo &device);

#endif // _VIDEOLIST_H_
