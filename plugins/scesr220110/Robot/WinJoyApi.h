#pragma once
#include <vector>
#include "robot_config.h"
#include "joystick_impl.h"

ROBOT_API const char *GetName();
ROBOT_API const char *GetNameAbbr();
ROBOT_API const char *GetVersion_();
ROBOT_API const char *GetModuleIcon();

/// <summary>
/// 创建设备接口, 注意: 最多支持两个手柄, 多个手柄需要多次调用Create
/// </summary>
/// <returns></returns>
ROBOT_API bool Create();

/// <summary>
/// 销毁设备接口, 注意: 多个手柄将在此全部销毁
/// </summary>
/// <returns></returns>
ROBOT_API bool Dispose();

/// <summary>
/// 启动设备, 注意: 多个手柄将在此全部启动(已经创建)
/// </summary>
/// <returns></returns>
ROBOT_API bool Start();

/// <summary>
/// 关闭设备, 注意: 多个手柄将在此全部关闭(已经创建)
/// </summary>
/// <returns></returns>
ROBOT_API bool Close();

/// <summary>
/// 设备是否正在运行
/// </summary>
/// <returns></returns>
ROBOT_API bool IsRunning();

/// <summary>
/// 获取NDI通道编号
/// </summary>
/// <param name="handles">返回通道编号</param>
/// <returns></returns>
ROBOT_API void GetHandles(std::vector<int> &handles);

/// <summary>
/// 获取手柄数据
/// </summary>
/// <param name="h">手柄编号, 通过GetHandles</param>
/// <param name="data"></param>
/// <param name="overtime"></param>
/// <returns></returns>
ROBOT_API bool GetJoyData(int h, joy_data &data, double overtime);
