#pragma once
#include <stdexcept>
#include <string>
#include "robot_config.h"

namespace ercp::error {
class normal_error : public std::runtime_error {
public:
    ROBOT_API_MEMBER explicit normal_error(std::string message);
};
}
