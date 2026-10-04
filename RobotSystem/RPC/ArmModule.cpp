#pragma once
#include <chrono>
#include <thread>

#include "robot_error.h"
#include "robot_settings.hpp"
#include "robot_devices.h"
#include "yunsbot_config.h"
#include "ArmModule.hpp"

namespace ercp {
extern bool PrepareForFollow();
extern bool PrepareStopFollow();

namespace rpc {
ArmModule::ArmModule()
    : FsmArm("ArmModule", t::A1_NotInit)
{
}

bool ArmModule::FollowStartCheck(const ex_trigger &) const
{
    if (!PrepareForFollow()) throw error::normal_error("Cannot start arm follow.");
    return true;
}

bool ArmModule::Initialize()
{
    return this->PostAsyncEvent(ex_signal{s::s_initialized});
}

bool ArmModule::DeInitialize()
{
    return ExecuteStop([this]() {
        this->m_state = state_t::A1_NotInit;
        return true;
    });
}

bool ArmModule::StartFollow()
{
    if (!IsRunning()) return false;
    // The PLC feedback is authoritative. If a previous command left the RPC
    // state at A5 while the arm is physically back at state 21, reconcile the
    // internal state before interpreting this click as "start follow".
    if (!SynchronizeWithBeckhoffFeedback()) {
        return false;
    }
    if (get_current_state() == state_t::A5_Following) return true;
    return this->PostAsyncEvent(ex_trigger{});
}

bool ArmModule::StopFollow()
{
    return ExecuteStop([this]() {
        if (!PrepareStopFollow()) return false;
        this->m_state = state_t::A4_Opened;
        return true;
    });
}

/**
 * @brief 功能：等待正在进行的状态转移结束，并把当前 Beckhoff 反馈同步到机械臂状态机。
 * @details 机制：先等待转移锁释放，再在 500 ms 窗口内读取设备运动状态并映射为 A3/A4/A5 等领域状态。
 */
bool ArmModule::SynchronizeWithBeckhoffFeedback()
{
    const auto feedback = GetRobot().BeckhoffArmMoveState();
    const auto current = get_current_state();

    arm_signal_t signal;
    state_t target;
    bool needs_sync = false;

    if (feedback == beckhoff_arm_move_state::BAMS_FOLDED &&
        (current == state_t::A2_Inited || current == state_t::A4_Opened ||
         current == state_t::A5_Following)) {
        signal = arm_signal_t::s_folded;
        target = state_t::A3_Folded;
        needs_sync = true;
    } else if (feedback == beckhoff_arm_move_state::BAMS_OPENED &&
               (current == state_t::A2_Inited || current == state_t::A3_Folded ||
                current == state_t::A5_Following)) {
        signal = arm_signal_t::s_opened;
        target = state_t::A4_Opened;
        needs_sync = true;
    } else if ((feedback == beckhoff_arm_move_state::BAMS_FOLLOWING ||
                feedback == beckhoff_arm_move_state::BAMS_FOLLOWED) &&
               (current == state_t::A2_Inited || current == state_t::A3_Folded ||
                current == state_t::A4_Opened)) {
        signal = arm_signal_t::s_following;
        target = state_t::A5_Following;
        needs_sync = true;
    }

    if (!needs_sync) {
        return true;
    }

    if (IsTaskBusy()) {
        return false;
    }

    if (IsTransition()) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(500);
        while (IsTransition() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (get_current_state() == target) {
            return true;
        }
        if (IsTransition()) {
            return false;
        }
    }

    if (!PostAsyncEvent(ex_signal{signal})) {
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        if (get_current_state() == target) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return get_current_state() == target;
}

//---------------------------------------------------------------------

bool ArmModule::IsConnected(state_t to)
{
    return FsmArm::IsConnected<ArmModule>(to);
}

/**
 * @brief 功能：请求机械臂状态机异步转移到指定目标状态。
 * @details 机制：检查当前状态与目标状态是否存在合法连接，存在时创建对应任务并提交给状态机。
 */
bool ArmModule::GotoState(state_t state)
{
    if (!IsRunning()) return false;
    // Reconcile stale RPC state with the physical Beckhoff feedback before
    // checking the transition graph. The reconciliation itself never writes
    // a PLC command.
    if (!SynchronizeWithBeckhoffFeedback()) {
        return false;
    }
    if (!IsConnected(state)) {
        ROBOT_ERROR(GetSettings().Basic.Verbose() > 0,
                    fmt::format("{} has no connection between {} and {}.",
                                GetName(),
                                GetStateName(this->get_current_state()),
                                GetStateName(state)))
        return false;
    }
    if (!IsTaskBusy()) {
        return PostTask(MakeTask(get_current_state(), state));
    }
    return false;
}

std::string ArmModule::GetStateName(const state_t &state) const
{
    return GetProcessName(state);
}

} // namespace rpc

} // namespace ercp
