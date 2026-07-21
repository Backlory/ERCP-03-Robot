#include <assert.h>
#include <algorithm>
#include <numeric>
#include <map>
#include <vector>

#include "robot_error.h"
#include "robot_config.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "yunsbot_config.h"
#include "ArmModule.hpp"
#include <mmsystem.h>
#include <Robot/YunSBot.h>
#pragma comment(lib, "winmm.lib")

namespace ercp {

#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

    namespace rpc {

        std::string rpc::GetProcessName(arm_state_t to)
        {
            // clang-format off
            static const std::map<arm_state_t, std::string> _mapping{
                { arm_state_t::A1_NotInit,      "A1_NotInit" },
                { arm_state_t::A2_Inited,       "A2_Inited" },
                { arm_state_t::A3_Folded,       "A3_Folded" },
                { arm_state_t::A4_Opened,       "A4_Opened" },
                { arm_state_t::A5_Following,    "A5_Following" },
            };
            // clang-format on
            if (_mapping.find(to) == _mapping.end()) {
                return fmt::format("{}", (int)to);
            } else {
                return _mapping.at(to);
            }
        }

        std::function<void()> ArmModule::MakeTask(const state_t &from, const state_t &to)
        {
            task::tasks_ptr<> ptr = nullptr;
            int verbose = ercp::GetSettings().Basic.Verbose();
            int t;

            if (to == t::A1_NotInit) {
                t = 1;
            } else if (to == t::A2_Inited) {
                t = 2;
            } else if (to == t::A3_Folded) {
                t = 3;
                auto seq = MoveBeckhoffTo(u8"折叠", false);
                seq->emplace(u8"转移", [this, verbose]() {
                    if (!this->PostAsyncEvent(ex_signal{ s::s_folded })) {
                        throw error::normal_error("transition failed.");
                    }
                    return true;
                });
                ptr = seq;
            } else if (to == t::A4_Opened) {
                t = 4;
                auto seq = MoveBeckhoffTo(u8"打开", true);
                seq->emplace(u8"转移", [this, verbose]() {
                    if (!this->PostAsyncEvent(ex_signal{ s::s_opened })) {
                        throw error::normal_error("transition failed.");
                    }
                    return true;
                });
                ptr = seq;
            } else if (to == t::A5_Following) {
                t = 5;
                auto seq = std::make_shared<task::SequentialTasks<>>(u8"启动跟随");
                seq->emplace(u8"设置跟随状态", [this, verbose]() {
                    return GetRobot().BeckhoffArmOperation(beckhoff_arm_operation::BAO_FOLLOW);
                });
                ptr = seq;
            } else {
                t = 6;
                auto seq = std::make_shared<task::SequentialTasks<>>(u8"请求转移");
                seq->emplace(u8"转移", [this, verbose]() {
                    if (!this->PostAsyncEvent(ex_signal{ s::s_opened })) {
                        throw error::normal_error("transition failed.");
                    }
                    return true;
                });
                ptr = seq;
            }
            ROBOT_INFO(true, t);
            return FsmArm::MakeTask(ptr);
        }

        task::seque_ptr<> ArmModule::MoveBeckhoffTo(std::string name, const bool bIsOpen)
        {
            int verbose = ercp::GetSettings().Basic.Verbose();
            auto seq = std::make_shared<task::SequentialTasks<>>(name);

            seq->emplace(u8"执行", [this, verbose, bIsOpen]() {
                auto &robot = ercp::GetRobot();
                return robot.BeckhoffMoveArmTo(bIsOpen);
            });

            seq->emplace(u8"等待", [this, verbose, bIsOpen]() {
                auto &robot = ercp::GetRobot();
                while (true) {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
                    int iState = robot.BeckhoffArmMoveState();
                    if ((bIsOpen && beckhoff_arm_move_state::BAMS_OPENED == iState)
                        || (!bIsOpen && beckhoff_arm_move_state::BAMS_FOLDED == iState)) {
                        break;
                    }
                }
                return true;
            });

            return seq;
        }

    } // namespace rpc

} // namespace ercp
