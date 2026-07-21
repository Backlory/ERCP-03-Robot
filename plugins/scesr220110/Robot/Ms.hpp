/**
 * @file YunSBot.Ms.Local。cpp
 * @author ytom (ybb331082@126.com)
 * @brief 机器人主从控制程序(远程版本)
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright (c) 2022 SIA-ILSR
 *
 */
#pragma once
#include <Poco/Net/UDPServer.h>
#include <Poco/Net/DatagramSocket.h>
using UdpClient = Poco::Net::DatagramSocket;
using UdpServer = Poco::Net::UDPServer;

#include "utils.h"
#include "Robot/VBot.hpp"
#include "yunsbot_config.h"

class LingMiServer : public Poco::Net::UDPHandler {
public:
    LingMiServer(Poco::Net::SocketAddress address);
    ilsr::mutex_data<robot_status> m_status;

private:
    Poco::Net::UDPHandler::List handlers;
    std::shared_ptr<UdpServer> server;
    void processData(char *buf) override;
    void processError(char *buf) override;
};

class LingMi : public VirtualRobot {
public:
    LingMi();

    bool GetStatus(robot_status &data, double overtime = 0.1) const;
    bool SendCmd(const control_cmd &cmd);

    void run() override;
    void close() override;
    bool is_online() const override;

public:
    //********************反馈********************//
    double get_bend_ud() override;
    double get_bend_lr() override;
    double get_rotation() override;
    double get_torque() override;
    double get_length() override;
    double get_q2() override;
    double get_q3() override;
    double get_q4() override;
    bool is_forwarding() override;

    bool ws_constraints(recog::operation_tag_t operation, int isfoward, double force,
        double image_quality, control_cmd &cmd);

private:
    ilsr::mutex_data<robot_status> m_status;
    std::shared_ptr<LingMiServer> server;
    UdpClient client;
};
