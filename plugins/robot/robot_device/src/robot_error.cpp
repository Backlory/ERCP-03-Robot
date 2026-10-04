#include "robot_error.h"

namespace ercp::error {
normal_error::normal_error(std::string message)
    : std::runtime_error(message.empty() ? "General runtime error." : message)
{
}
}
