#pragma once
#include "robot_error.h"
#include "robot_settings.hpp"
#include "ArmModule.hpp"

namespace ercp {
    extern bool PrepareForFollow();
    extern bool PrepareStopFollow();

    namespace rpc {
        ArmModule::ArmModule()
            : FsmArm("ArmModule", t::A1_NotInit, 0.008, 1.0, GetSettings().Basic.Verbose())
        {
        }

        bool ArmModule::FollowStartCheck(const ex_trigger &) const
        {
            return PrepareForFollow();
        }

        bool ArmModule::FollowStopCheck(const ex_trigger &) const
        {
            return PrepareStopFollow();
                    }

        bool ArmModule::Initialize() { return this->PostAsyncEvent(ex_signal{ s::s_initialized }); }

        bool ArmModule::DeInitialize()
        {
            return this->PostAsyncEvent(ex_signal{ s::s_deinitialized });
        }

        bool ArmModule::StartFollow() { return this->PostAsyncEvent(ex_trigger{}); }

        bool ArmModule::StopFollow() { return this->PostAsyncEvent(ex_trigger{}); }

        //---------------------------------------------------------------------

        bool ArmModule::IsConnected(state_t to) { return FsmArm::IsConnected<ArmModule>(to); }

        bool ArmModule::GotoState(state_t state)
        {
            if (!IsConnected(state)) {
                ROBOT_ERROR(GetSettings().Basic.Verbose() > 0,
                    fmt::format("{} has no connection between {} and {}.", GetName(),
                        GetStateName(this->get_current_state()), GetStateName(state)))
                return false;
            }
            if (!IsTaskBusy()) {
                return PostTask(MakeTask(get_current_state(), state));
            }
            return false;
        }

        bool ArmModule::OnError(const transition_error &error)
        {
            try {
                boost::rethrow_exception(error.except_ptr);

            } catch (error::action::action_error &e) {
                ROBOT_INFO(_verbose > 0, e.what());
                if (m_retry < 3 && error::solution(e)) {
                    ROBOT_INFO(_verbose > 1, "solution true");
                    // Cause async retry the state.
                    auto state = get_current_state();
                    // OnPostTransition(state, state);
                    m_retry++;
                    ClearFaults();
                    PostTask(FsmArm::MakeTask(e.m_task));
                    return true;
                }
                ROBOT_INFO(_verbose > 1, "solution false");
            } catch (std::exception e) {
            }
            m_retry = 0;
            return false;
        }

        bool ArmModule::OnRescue(const transition_rescue &) { return true; }

        int ArmModule::GetErrorCode(const boost::exception_ptr &_exception) const { return -1; }

        std::string ArmModule::GetStateName(const state_t &state) const
        {
            return GetProcessName(state);
        }

    } // namespace rpc

} // namespace ercp
