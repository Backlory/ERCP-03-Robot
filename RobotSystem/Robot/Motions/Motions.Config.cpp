#include <map>
//#include "cml_driver.hpp"
#include "robot_config.h"
#include "yunsbot_config.h"
#include "robot_settings.hpp"

using namespace device;

namespace ercp {

    extern const double M1_Fold = (-13042.0); // 相对于上电初始码盘值
    extern const double M2_Fold = (-13440.0); // 相对于上电初始码盘值
    extern const double M3_Fold = (37049.0); // 相对于上电初始码盘值
    extern const double M4_Fold = (-52332.0); // 相对于上电初始码盘值

    extern const double CUTTER_NEGLIM = (300);
    extern const double CUTTER_POSLIM = (2000);
    extern const double CUTTER_SPEED_RATIO = (10.0);

    extern const double STEP_SPEED = 1.0 /*mm*/;
    extern const double STEP_SPEED_RATIO = 10.0 /*mm/s*/;

    extern const std::map<motor_t, cml::JointMotorParam> JointConfigs_Auhua;
    extern const std::map<motor_t, cml::JointMotorParam> JointConfigs_AuhuaDeuodenum;

    const std::map<motor_t, cml::JointMotorParam> &GetJointConfigs(ScopeType_t scope)
    {
        if (scope == ScopeType_t::Olympus_H260) {
            return JointConfigs_AuhuaDeuodenum;
        }
        return JointConfigs_Auhua;
    }

    const std::map<motor_t, double> BrakeCurrent = {
        //
        { motor_t::arm_1, 5.0 }, // 0.25
        { motor_t::arm_2, 5.0 }, // 1.2
        { motor_t::arm_3, 5.0 }, // 1.0
        { motor_t::arm_4, 5.0 }, // 1.5
    };

    const std::map<motor_t, CML::COPLEY_HOME_METHOD> HomeMethod = {
        { motor_t::feed_move, CML::CHM_NONE },
        { motor_t::feed_clamp, CML::CHM_NONE },
        { motor_t::oper_big, CML::CHM_PLIM },
        { motor_t::oper_small, CML::CHM_PLIM },
        { motor_t::oper_pincer, CML::CHM_PLIM },
        { motor_t::cutter_rot, CML::CHM_NONE },
        { motor_t::cutter_feed, CML::CHM_NONE },
    };

    const std::map<motor_t, double> HomeOffset = {
        { motor_t::oper_big, -317.45 },
        { motor_t::oper_small, -259.5 },
        { motor_t::oper_pincer, -66.0 },
    };

    const std::set<motor_t> ZeroList = {
        motor_t::oper_rotate,
        motor_t::feed_clamp,
    };

    const cml::JointMotorParam &GetJointConfig(motor_t mid)
    {
        //
        return GetJointConfigs((ScopeType_t)GetSettings().Device.ScopeType()).at(mid);
    }

    bool IsNeedBrake(motor_t mid)
    {
        //
        return BrakeCurrent.find(mid) != BrakeCurrent.end();
    }

    bool IsNeedZero(motor_t mid)
    {
        //
        return ZeroList.find(mid) != ZeroList.end();
    }

    double GetBrakeCurrent(motor_t mid)
    {
        //
        return (BrakeCurrent.find(mid) != BrakeCurrent.end()) ? BrakeCurrent.at(mid) : -1;
    }

    std::shared_ptr<CML::COPLEY_HOME_METHOD> GetHomeMethod(motor_t mid)
    {
        return (HomeMethod.find(mid) != HomeMethod.end())
            ? std::make_shared<CML::COPLEY_HOME_METHOD>(HomeMethod.at(mid))
            : nullptr;
    }

    std::shared_ptr<double> GetHomeOffset(motor_t mid)
    {
        double add = 0;
        if (mid == motor_t::oper_big)
            add = GetSettings().Device.Offset.Big();
        if (mid == motor_t::oper_small)
            add = GetSettings().Device.Offset.Small();

        auto d = (HomeOffset.find(mid) != HomeOffset.end())
            ? std::make_shared<double>(HomeOffset.at(mid))
            : nullptr;
        if (d)
            *d += add;
        return d;
    }

} // namespace ercp
