#include <stdint.h>
#include <assert.h>
#include <locale>
#include "utils.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "Robot/YunSBot.h"
#include "Net/server.h"

// extern int AfxMain();
//cylTest
int main(int argv, char *argc[])
{
    std::setlocale(LC_ALL, "zh_CN.UTF-8");

    // 加载配置
    {
        int error = ercp::LoadSettings();
        if (error < 0) {
            MessageBoxW(NULL, L"加载配置文件失败.", L"错误", NULL);
            exit(-1);
        } else if (error == 0
            && MessageBoxW(
                   NULL, L"找不到配置文件 config.yaml! 是否生成并使用默认参数?", L"错误", MB_YESNO)
                == IDYES) {
            ercp::SaveSettings();
        }
    }

    srand((unsigned int)time(0));

#if USING_LOGURU
    auto logfile = ercp::GetLogPath() + "\\ercp.log";
    loguru::add_file(logfile.c_str(), loguru::Append, loguru::Verbosity_MAX);
#endif

    // 初始化定时器
    ilsr::Time::GetInstance();

    // 初始化机器人
    auto &robot = ercp::GetRobot();
    auto &ercp = ercp::YunSBot::GetInstance();

    // 初始化服务端
    return HttpServer(7998, ercp::GetSettings().Basic.Verbose()).run(argv, argc);
}
