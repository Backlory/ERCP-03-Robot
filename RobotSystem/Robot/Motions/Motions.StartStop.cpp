#include <assert.h>
#include <vector>
#include "task.hpp"
#include "robot_config.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "yunsbot_config.h"
//#include "cml_driver.hpp"
#include "serial/stepm_driver.hpp"

using namespace task;

namespace ercp {

    extern const cml::JointMotorParam &GetJointConfig(motor_t mid);
    extern bool IsNeedBrake(motor_t mid);
    extern bool IsNeedZero(motor_t mid);
    extern double GetBrakeCurrent(motor_t mid);
    extern std::shared_ptr<CML::COPLEY_HOME_METHOD> GetHomeMethod(motor_t mid);
    extern std::shared_ptr<double> GetHomeOffset(motor_t mid);

    extern const double CUTTER_NEGLIM;
    extern const double CUTTER_POSLIM;

#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))


} // namespace ercp
