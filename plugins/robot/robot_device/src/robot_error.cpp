#define _USE_MATH_DEFINES
#include <cmath>
#include <cassert>
#include "task.hpp"
#include "robot_config.h"
#include "robot_error.h"

namespace ercp {

    std::string do_report(const task::report_t &rep)
    {
        std::string info;
        for (auto &r : rep) {
            info += fmt::format("{}, {}\r\n", r.first, r.second);
        }
        return info;
    }

    namespace error {

        ROBOT_API_MEMBER normal_error::normal_error(std::string msg)
            : std::exception(msg.empty() ? "General runtime error." : msg.c_str())
        {
        }

        ROBOT_API_MEMBER fatal_error::fatal_error(std::string msg)
            : std::exception(msg.empty() ? "Fatle runtime error." : msg.c_str())
        {
        }

        ROBOT_API_MEMBER gpio_error::gpio_error(std::string msg)
            : std::exception(fmt::format("GPIO work error. {}", msg).c_str())
        {
        }

        namespace motor {

            ROBOT_API_MEMBER initial_error::initial_error(motor_t index, std::string msg)
                : m_id(index)
                , std::exception(msg.c_str())
            {
            }

            ROBOT_API_MEMBER command_error::command_error(motor_t index)
                : m_id(index)
                , std::exception("Send command failed.")
            {
            }

            ROBOT_API_MEMBER motion_error::motion_error(motor_t index)
                : m_id(index)
                , std::exception("Motor cannot move to target.")
            {
            }

        } // namespace motor

        namespace action {
            ROBOT_API_MEMBER action_error::action_error(
                const task::tasks_ptr<> &task, const task::report_t &report)
                : m_task(task)
                , m_report(report)
                , std::exception(("Action group running failed: \r\n" + do_report(report)).c_str())
            {
            }
        } // namespace action

        ROBOT_API_MEMBER bool solution(const normal_error &e)
        {
            // Retry
            return true;
        }

        ROBOT_API_MEMBER bool solution(const fatal_error &e)
        {
            // Retry
            return false;
        }

        ROBOT_API_MEMBER bool solution(const gpio_error &e)
        {
            // Retry
            return true;
        }

        ROBOT_API_MEMBER bool solution(const motor::command_error &e)
        {
            // Retry
            return true;
        }

        ROBOT_API_MEMBER bool solution(const motor::motion_error &e)
        {
            // auto &robot = _RobotDevice::GetInstance();
            // auto &motor = robot.GetMotor(e.m_id);

            // if (!motor) {
            //    return false;
            //}

            // int count = 0;
            // do {
            //    motor->EnableMotor(true);
            //    std::this_thread::sleep_for(std::chrono::milliseconds(500));
            //    if (motor->IsInitialized() && motor->IsSoftwareEnabled()) {
            //        return true;
            //    }
            //    count++;
            //} while (count < 3);
            // return false;
            return true;
        }

        ROBOT_API_MEMBER bool solution(const action::action_error &e)
        {
            auto &rep = e.m_report;
            for (auto ite = rep.begin(); ite != rep.end(); ++ite) {
                if (ite->second == task::task_status::errored) {
                    auto ptr = e.m_task->get_task(ite->first);
                    assert(ptr);
                    auto err = ptr->get_error();
                    assert(err);
                    try {
                        std::rethrow_exception(err);
                    } catch (normal_error &e) {
                        return solution(e);
                    } catch (gpio_error &e) {
                        return solution(e);
                    } catch (motor::command_error &e) {
                        return solution(e);
                    } catch (motor::motion_error &e) {
                        return solution(e);
                    } catch (action::action_error &e) {
                        return solution(e);
                    } catch (std::exception &e) {
                        return false;
                    }
                }
            }
            return true;
        }

        ROBOT_API_MEMBER bool solution(const std::exception &e)
        {
            // Not defined error.
            return false;
        }

    } // namespace error

} // namespace ercp
