#pragma once
#include "task.hpp"
#include "robot_config.h"

enum class motor_t;

namespace ercp {

    namespace error {

        class normal_error : public std::exception {
        public:
            ROBOT_API_MEMBER normal_error(std::string msg);
        };

        class fatal_error : public std::exception {
        public:
            ROBOT_API_MEMBER fatal_error(std::string msg);
        };

        class gpio_error : public std::exception {
        public:
            ROBOT_API_MEMBER gpio_error(std::string msg);
        };

        namespace motor {

            class initial_error : public std::exception {
            public:
                ROBOT_API_MEMBER initial_error(motor_t index, std::string msg);
                const motor_t m_id;
            };

            class command_error : public std::exception {
            public:
                ROBOT_API_MEMBER command_error(motor_t index);
                const motor_t m_id;
            };

            class motion_error : public std::exception {
            public:
                ROBOT_API_MEMBER motion_error(motor_t index);
                const motor_t m_id;
            };

        } // namespace motor

        namespace action {

            class action_error : public std::exception {
            public:
                ROBOT_API_MEMBER action_error(
                    const task::tasks_ptr<> &task, const task::report_t &report);
                const task::tasks_ptr<> m_task;
                const task::report_t m_report;
            };

        } // namespace action

        ROBOT_API_MEMBER bool solution(const normal_error &e);
        ROBOT_API_MEMBER bool solution(const gpio_error &e);
        ROBOT_API_MEMBER bool solution(const motor::command_error &e);
        ROBOT_API_MEMBER bool solution(const motor::motion_error &e);
        ROBOT_API_MEMBER bool solution(const action::action_error &e);
        ROBOT_API_MEMBER bool solution(const std::exception &e);

    } // namespace error

    ROBOT_API_MEMBER std::string do_report(const task::report_t &rep);

} // namespace ercp
