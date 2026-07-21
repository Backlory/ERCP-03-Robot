#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "utils.h"
#include "../CV/cvtask.hpp"
#include "../CV/recog.hpp"
#include "VBot.hpp"

VirtualRobot *VirtualRobot::m_instance = nullptr;

VirtualRobot::VirtualRobot() { m_instance = this; }

void VirtualRobot::run()
{
    if (m_thread) {
        close();
    }
#ifdef DEBUG
    std::cout << "Start robot ..." << std::endl;
#endif // DEBUG
    m_thread.swap(std::make_shared<boost::thread>(boost::bind(&VirtualRobot::runner, this)));
}

void VirtualRobot::close()
{
    if (m_thread) {
        m_thread->interrupt();
        m_thread->join();
#ifdef DEBUG
        std::cout << "Stop robot." << std::endl;
#endif // DEBUG
    }
    m_thread.reset();
}

int VirtualRobot::runner() { return 0; }

bool VirtualRobot::is_online() const { return m_thread && m_thread->joinable(); }

double VirtualRobot::get_bend_ud() { return 90 * std::sin(ilsr::Time::wall_time()); }

double VirtualRobot::get_bend_lr() { return 90 * std::cos(ilsr::Time::wall_time()); }

double VirtualRobot::get_rotation() { return 0; }

double VirtualRobot::get_torque() { return 8 * std::cos(ilsr::Time::wall_time()); }

double VirtualRobot::get_length() { return 1000 * std::cos(ilsr::Time::wall_time()); }

double VirtualRobot::get_q2() { return 0; }

double VirtualRobot::get_q3() { return 0; }

double VirtualRobot::get_q4() { return 0; }

bool VirtualRobot::is_forwarding() { return true; }

bool VirtualRobot::ws_constraints(int operation, int isfoward, double force, double image_quality,
    const VirtualRobot::direct_t &dire)
{
    return true;
}
