#pragma once
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
#include "cml_driver.hpp"
#include "ArmModule.hpp"
#include <mmsystem.h>
#include <Robot/YunSBot.h>
#pragma comment(lib, "winmm.lib")

namespace ercp {

    extern const Vector3d Trajectory(const double &length, const Vector3d &stored);

#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

    namespace rpc {

        std::string rpc::GetProcessName(arm_state_t to)
        {
            // clang-format off
            static const std::map<arm_state_t, std::string> _mapping{
                { arm_state_t::A1_NotInit,      "A1_NotInit" },
                { arm_state_t::A2_Inited,      	"A2_Inited" },
                { arm_state_t::A3_Folded,    	"A3_Folded" },
                { arm_state_t::A4_Opened,    	"A4_Opened" },
                { arm_state_t::A5_Following,    "A5_Following" },
            };
            // clang-format on
            if (_mapping.find(to) == _mapping.end()) {
                return fmt::format("{}", (int)to);
            } else {
                return _mapping.at(to);
            }
        }

        //// 判断机械臂已经到达打开位置
        // bool ArmModule::IsAtOpened(double eps)
        //{
        //    auto &robot = ercp::GetRobot();
        //    auto arm2 = robot.GetMotor(motor_t::arm_2);
        //    auto arm3 = robot.GetMotor(motor_t::arm_3);
        //    auto arm4 = robot.GetMotor(motor_t::arm_4);
        //    if (!arm2 || !arm3 || !arm4) {
        //        return false;
        //    }
        //    Vector3d p(arm2->GetPosition(), arm3->GetPosition(), arm4->GetPosition());
        //    const Vector3d j = Trajectory(0, Vector3d::Zero());
        //    return (p - j).norm() < eps;
        //}

        ///////////////////////////////////////////////////////////////////////////

        std::function<void()> ArmModule::MakeTask(const state_t &from, const state_t &to)
        {
            task::tasks_ptr<> ptr = nullptr;
            int verbose = ercp::GetSettings().Basic.Verbose();


            int t;



            if (to == t::A1_NotInit) {
                t = 1;
            }

            else if (to == t::A2_Inited) {
                t = 2;
            }

             else if (to == t::A3_Folded) {
                t = 3;
                //Vector3d j = Vector3d::Zero();
                //j(0) = 90;
                //j(1) = -90;
                //j(2) = 0;
                //auto seq = MoveArmTo(u8"折叠", j);
                auto seq = MoveBeckhoffTo(u8"折叠", false);
                seq->emplace(u8"转移", [this, verbose]() {
                    if (!this->PostAsyncEvent(ex_signal{ s::s_folded })) {
                        throw error::normal_error("transition failed.");
                    }
                    return true;
                });
                ptr = seq;
            }

            else if (to == t::A4_Opened) {
                t = 4;
                //Vector3d j = Vector3d::Zero();

                //j = Trajectory(0, Vector3d::Zero());
                //auto seq = MoveArmTo(u8"打开", j);
                auto seq = MoveBeckhoffTo(u8"打开", true);
                //seq->emplace(u8"清除编码", [this, verbose]() {
                //    auto m = GetRobot().GetMotor(motor_t::feed_move);
                //    if (m && m->api.SetPositionLoad(0)) {
                //        return true;
                //    }
                //    return true;
                //});
                seq->emplace(u8"转移", [this, verbose]() {
                    if (!this->PostAsyncEvent(ex_signal{ s::s_opened })) {
                        throw error::normal_error("transition failed.");
                    }
                    return true;
                });
                ptr = seq;
            }

            else if (to == t::A5_Following) {
                t = 5;
                auto seq = std::make_shared<task::SequentialTasks<>>(u8"启动跟随");
                seq->emplace(u8"设置跟随状态", [this, verbose]() { return GetRobot().BeckhoffArmOperation(beckhoff_arm_operation::BAO_FOLLOW); });
                ptr = seq;

                // auto seq = std::make_shared<task::SequentialTasks<>>(u8"启动跟随");
                //// seq->emplace(u8"读取", [this, verbose]() { return true; });
                //// seq->emplace(u8"规划", [this, verbose]() { return true; });
                // seq->emplace(u8"运动", [this, verbose]() {
                //    if (!this->PostAsyncEvent(ex_signal{ s::s_opened })) {
                //        throw error::normal_error("transition failed.");
                //    }
                //    return true;
                //});
                // ptr = seq;
            }

            else {
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

        task::seque_ptr<> ArmModule::MoveBeckhoffTo(std::string name, const bool bIsOpen) {
            int verbose = ercp::GetSettings().Basic.Verbose();
            auto seq = std::make_shared<task::SequentialTasks<>>(name);

            // 执行操作
            seq->emplace(u8"执行", [this, verbose, bIsOpen]() {
                auto &robot = ercp::GetRobot();
                return robot.BeckhoffMoveArmTo(bIsOpen);
            });

            // 等待结果
            seq->emplace(u8"等待", [this, verbose, bIsOpen]() {
                auto &robot = ercp::GetRobot();

                // 等待正确反馈
                while ( true ) {
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

                    int iState = robot.BeckhoffArmMoveState();
                    if ((bIsOpen && beckhoff_arm_move_state::BAMS_OPENED == iState)
                        || (!bIsOpen && beckhoff_arm_move_state::BAMS_FOLDED == iState)) break;
                }

                return true;
            });

            return seq;
        }


        ///// 关节单位: 度
        //task::seque_ptr<> ArmModule::MoveArmTo(std::string name, const Vector3d joints)
        //{
        //    int verbose = ercp::GetSettings().Basic.Verbose();
        //    auto seq = std::make_shared<task::SequentialTasks<>>(name);
        //    seq->emplace(u8"读取", [this, verbose]() {
        //        auto &robot = ercp::GetRobot();
        //        const std::vector<cml::motor_ptr> arms{
        //            robot.GetMotor(motor_t::arm_1),
        //            robot.GetMotor(motor_t::arm_2),
        //            robot.GetMotor(motor_t::arm_3),
        //        };
        //        // 检查设备
        //        for (const auto &a : arms) {
        //            if (!a || !a->IsOperable()) {
        //                throw error::fatal_error(u8"关节电机异常, 无法进行打开.");
        //            }
        //        }
        //        return true;
        //    });
        //    seq->emplace(u8"执行", [this, verbose, joints]() {
        //        auto &robot = ercp::GetRobot();
        //        const std::vector<cml::motor_ptr> arms{
        //            robot.GetMotor(motor_t::arm_1),
        //            robot.GetMotor(motor_t::arm_2),
        //            robot.GetMotor(motor_t::arm_3),
        //        };
        //        // 读取当前位置和速度
        //        Vector3d pos0 = Vector3d::Zero();
        //        Vector3d vel = Vector3d::Zero();
        //        for (int i = 0; i < 3; ++i) {
        //            pos0[i] = arms[i]->GetPosition();
        //            vel[i] = arms[i]->GetParam()->get_speed(false);
        //            // 预期最迟8秒完成折叠
        //            vel[i] = std::abs(joints[i] - pos0[i]) / 5.0 * vel[i];
        //            vel[i] = std::clamp(vel[i], 0.25, 1.0);
        //        }
        //        // 规划
        //        ROBOT_INFO(verbose > 0,
        //            fmt::format("Arm targeting: {}, {}, {}; Speed: {}, {}, {}  ...\n",
        //                joints[0], joints[1], joints[2], vel[0], vel[1], vel[2]));
        //        bool succ = true;
        //        for (int i = 0; i < 3; ++i) {
        //            succ = succ && arms[i]->MoveAbs(joints[i], vel[i]);
        //        }
        //        return succ;
        //    });
        //    seq->emplace(u8"等待", [this, verbose]() {
        //        auto &robot = ercp::GetRobot();
        //        const std::vector<cml::motor_ptr> arms{
        //            robot.GetMotor(motor_t::arm_1),
        //            robot.GetMotor(motor_t::arm_2),
        //            robot.GetMotor(motor_t::arm_3),
        //        };
        //        bool finish = false;
        //        while (!finish) {
        //            finish = //
        //                (arms[0]->api.GetEvent() & CML::AMPEVENT_MOVEDONE == CML::AMPEVENT_MOVEDONE)
        //                && (arms[1]->api.GetEvent()
        //                    & CML::AMPEVENT_MOVEDONE == CML::AMPEVENT_MOVEDONE)
        //                && (arms[2]->api.GetEvent()
        //                    & CML::AMPEVENT_MOVEDONE == CML::AMPEVENT_MOVEDONE);
        //            sleep_ms(100);
        //        }
        //        ROBOT_INFO(verbose > 1, u8"机械臂已折叠\n");
        //        return true;
        //    });
        //    return seq;
        //}

    } // namespace rpc

} // namespace ercp
