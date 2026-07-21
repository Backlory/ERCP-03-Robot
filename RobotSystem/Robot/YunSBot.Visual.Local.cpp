/**
 * @file YunSBot.Visual.Local.cpp
 * @author ytom (ybb331082@126.com)
 * @brief 机器人视觉功能程序(本地版本)
 * @version 0.1
 * @date 2022-04-03
 *
 * @copyright Copyright (c) 2022 SIA-ILSR
 *
 */
#include "net/udp_client.hpp"
#include "robot_settings.hpp"
#include "YunSBot.h"

namespace net {
    namespace udp {

        template <>
        bool cmd_client::decode(cmd_client::pack_type &packet, cmd_client::type &dst)
        {
            dst = std::move(packet);
            return true;
        }

        template <>
        void cmd_client::enter()
        {
        }

        template <>
        void cmd_client::exit()
        {
        }

    } // namespace udp
} // namespace net

namespace ercp {

    YunSBot::_visual::_visual(YunSBot &p)
        : parent(p)
    {
        m_client = std::make_shared<net::udp::cmd_client>("192.168.1.100:13005", 1);        // NavigateSystem云端ip
    }

    bool YunSBot::_visual::IsVisionOnline() const
    {
        control_cmd cmd;
        return m_client && m_client->frame.TryGet(cmd, 150);
    }

    bool YunSBot::_visual::GetVisionCmd(control_cmd &cmd, double overtime) const
    {
        return m_client && m_client->frame.TryGet(cmd, overtime);
    }

} // namespace ercp
