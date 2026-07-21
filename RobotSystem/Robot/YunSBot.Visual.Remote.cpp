/**
 * @file YunSBot.Visual.Remote.cpp
 * @author ytom (ybb331082@126.com)
 * @brief 机器人视觉功能程序(远程版本)
 * @version 0.1
 * @date 2022-04-03
 *
 * @copyright Copyright (c) 2022 SIA-ILSR
 *
 */
#include "YunSBot.h"
#include "RemoteManager.h"

namespace ercp {

    YunSBot::_visual::_visual(YunSBot &p)
        : parent(p)
    {
    }

    bool YunSBot::_visual::IsVisionOnline() const
    {
        return RemoteManager::GetInstance().IsVisionOnline();
    }

} // namespace ercp
