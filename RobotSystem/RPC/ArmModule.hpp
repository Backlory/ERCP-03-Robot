#pragma once
#include "Module.hpp"

namespace ercp {

    namespace rpc {

        enum class arm_state_t {
            A1_NotInit,
            A2_Inited,
            A3_Folded,
            A4_Opened,
            A5_Following,
        };

        enum class arm_signal_t {
            s_initialized,
            s_folded,
            s_opened,
            s_deinitialized,
        };

        std::string GetProcessName(arm_state_t state);

        using FsmArm = module::Module<arm_state_t, int, true>;

        class ArmModule : public FsmArm {

            friend class fsmlib::fsm<arm_state_t>;

        public:
            static ArmModule &GetInstance()
            {
                static ArmModule me;
                return me;
            }

        public:
            bool GotoState(state_t state);
            bool Initialize();
            bool DeInitialize();
            bool StartFollow();
            bool StopFollow();

        public:
            std::string GetStateName(const state_t &state) const override;

        private:
            template <typename Event>
            bool PostAsyncEvent(const Event &event)
            {
                return FsmArm::PostAsyncEvent<ArmModule>(event);
            }

            std::function<void()> MakeTask(const state_t &, const state_t &);

            bool IsConnected(state_t to);
            bool OnError(const transition_error &) override;
            bool OnRescue(const transition_rescue &) override;
            int GetErrorCode(const boost::exception_ptr &_exception) const override;

        protected:
            ArmModule();
            ArmModule(const ArmModule &) = delete;

            //task::seque_ptr<> MoveArmTo(std::string name, const Eigen::Vector3d joints);

            task::seque_ptr<> MoveBeckhoffTo(std::string name, const bool bIsOpen);

        protected:
            ///< [Ex]teral signal to start current sub-module.
            struct ex_trigger {
            };
            ///< External signal of some switch signal arrived.
            struct ex_signal {
                arm_signal_t signal;
            };

        protected:
            template <arm_signal_t sig>
            bool SignalCheck(const ex_signal &s) const
            {
                return s.signal == sig;
            }

            bool FollowStartCheck(const ex_trigger &) const;
            bool FollowStopCheck(const ex_trigger &) const;

        protected:
            // Adaptive for secondary derived class
            template <state_t from, typename Event, state_t to,
                void (ArmModule::*action)(const Event &) = nullptr,
                bool (ArmModule::*guard)(const Event &) const = nullptr>
            using transition_sp = transition_xe<ArmModule, from, Event, to, action, guard>;

            // clang-format off
            using t = state_t;
            using s = arm_signal_t;
            using transition_table = fsm_table<
            //             From                 Event               To                   Action      Guard (optional)
            //  -----------+--------------------+-------------------+--------------------+-----------+-----------------+-
            transition_sp< t::A1_NotInit,        ex_signal,          t::A2_Inited,       nullptr,  &ArmModule::SignalCheck<s::s_initialized>>,
            transition_sp< t::A2_Inited,         ex_signal,          t::A3_Folded,       nullptr,  &ArmModule::SignalCheck<s::s_folded>>,
            transition_sp< t::A2_Inited,         ex_signal,          t::A4_Opened,       nullptr,  &ArmModule::SignalCheck<s::s_opened>>,
            transition_sp< t::A3_Folded,         ex_signal,          t::A4_Opened,       nullptr,  &ArmModule::SignalCheck<s::s_opened>>,
            transition_sp< t::A4_Opened,         ex_trigger,         t::A5_Following,    nullptr,  &ArmModule::FollowStartCheck>,
            transition_sp< t::A2_Inited,         ex_trigger,          t::A5_Following,      nullptr,  &ArmModule::FollowStartCheck>,
            transition_sp< t::A5_Following,      ex_trigger,         t::A4_Opened,       nullptr,  &ArmModule::FollowStopCheck>,
            transition_sp< t::A4_Opened,         ex_signal,          t::A3_Folded,       nullptr,  &ArmModule::SignalCheck<s::s_folded>>,
            transition_sp< t::A3_Folded,         ex_signal,          t::A2_Inited,       nullptr,  &ArmModule::SignalCheck<s::s_initialized>>,
            transition_sp< t::A2_Inited,         ex_signal,          t::A1_NotInit,      nullptr,  &ArmModule::SignalCheck<s::s_deinitialized>>,
            transition_sp< t::A3_Folded,         ex_signal,          t::A1_NotInit,      nullptr,  &ArmModule::SignalCheck<s::s_deinitialized>>
            
            //  -----------+--------------------+-------------------+--------------------+-----------+-----------------+-
            >;
            // clang-format on
        };

    } // namespace rpc

} // namespace ercp