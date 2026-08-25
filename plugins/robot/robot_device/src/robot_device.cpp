#include <boost/filesystem.hpp>
#include "robot_device_inner.hpp"

namespace fs = boost::filesystem;

namespace ercp {

ROBOT_API_MEMBER RobotDevice &GetRobot()
{
    return _RobotDevice::GetInstance();
}

std::string GenerateFolder()
{
    auto time = ilsr::Time::GetInstance().logtime();
    fs::path dir = fs::current_path() / "log" / "robot" / time;
    return dir.string();
}

ROBOT_API_MEMBER std::string GetLogPath()
{
    static const std::string m_log_path(GenerateFolder());
    // 创建日志文件夹
    if (!fs::exists(m_log_path)) {
        fs::create_directories(m_log_path);
    }
    return m_log_path;
}

///////////////////////////////////////////////////////////////////////////

_RobotDevice::_RobotDevice()
{
    // 创建日志文件
#if USING_LOGURU
    auto logfile = GetLogPath() + "\\ercp_devices.log";
    loguru::add_file(logfile.c_str(), loguru::Append, loguru::Verbosity_MAX);
#endif
    InitDevices(GetSettings().Basic.Verbose());
}

/**
 * @brief 根据配置初始化 Beckhoff 设备连接。
 * @details 最多重试五次；连接仍失败时保留离线状态，由上层生命周期和状态轮询继续处理，而不在初始化阶段阻塞退出。
 */
void _RobotDevice::InitDevices(int verbose)
{
    // 按配置建立 Beckhoff 驱动连接，最多重试五次；连接失败只保留离线状态，由上层在后续周期继续处理。
    auto &settings = GetSettings();

    // 倍福驱动初始化
    {
        // 反复保证链接
        bool bIsBeckhoffRun = false;

        for (int i = 0; i < 5; i++) {
            if (bIsBeckhoffRun = beckhoff::Beckhoff_Motor::GetInstance().OpenConn(
                    settings.Device.Beckhoff.Addr(),
                    settings.Device.Beckhoff.Port(),
                    settings.Device.Beckhoff.Transport(),
                    settings.Device.Beckhoff.TcpHost(),
                    settings.Device.Beckhoff.TcpPort()))
                break;

            boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
        }

        // 倍福没有链接上，不在继续
        // assert(bIsBeckhoffRun);
    }
}

_RobotDevice::~_RobotDevice() = default;

///////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////

double _RobotDevice::GetHandleForce(double overtime) const
{
    //double f = 0;
    //m_handle_force && m_handle_force->GetForce(f, overtime);
    // return f;

    return beckhoff::Beckhoff_Motor::GetInstance().Force(2);
}

double _RobotDevice::GetScopeForce(double overtime) const
{
    //std::array<double, 1> _tmp;
    //return (m_scope_force && m_scope_force->GetData(_tmp, overtime)) ? _tmp[0] : 0;
    return beckhoff::Beckhoff_Motor::GetInstance().DeliverForce();
}

double _RobotDevice::GetScopeTorque(double overtime) const
{
    //std::array<double, 1> _tmp;
    //return (m_scope_torque && m_scope_torque->GetData(_tmp, overtime)) ? _tmp[0] : 0;
    return beckhoff::Beckhoff_Motor::GetInstance().Force(2);
}

double _RobotDevice::GetCannulaForce(double overtime) const
{
    //std::array<double, 1> _tmp;
    //return (m_cannula_force && m_cannula_force->GetData(_tmp, overtime)) ? _tmp[0] : 0;
    return beckhoff::Beckhoff_Motor::GetInstance().Force(2);
}

double _RobotDevice::GetWireForce(double overtime) const
{
    //std::array<double, 1> _tmp;
    //return (m_wire_force && m_wire_force->GetData(_tmp, overtime)) ? _tmp[0] : 0;
    return beckhoff::Beckhoff_Motor::GetInstance().Force(2);
}

bool _RobotDevice::BeckhoffMoveArmTo(bool bIsOpen) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().MoveArmTo(bIsOpen);
}

beckhoff_arm_move_state _RobotDevice::BeckhoffArmMoveState() const
{
    return beckhoff::Beckhoff_Motor::GetInstance().MoveState();
    ;
}

std::uint32_t _RobotDevice::BeckhoffWriteFollowCommand(
    const beckhoff_follow_cmd &command) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().FollowOperationDataResult(
        sizeof(command), &command);
}

std::uint32_t _RobotDevice::BeckhoffGoldDiscreteCommandResult(
    const device::beckhoff::GoldDiscreteCommand &command) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().GoldDiscreteCommandResult(command);
}

bool _RobotDevice::BeckhoffArmOperation(beckhoff_arm_operation iArmOper) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().ArmOperation(iArmOper);
}

bool _RobotDevice::BeckhoffReadAsexPos(double asex_pos[19]) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().ReadAsexPos(asex_pos);
}

double _RobotDevice::BeckhoffForce(INT16 iPos) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().Force(iPos);
}

bool _RobotDevice::IsOpen() const
{
    return beckhoff::Beckhoff_Motor::GetInstance().IsOpen();
}

device::beckhoff::BeckhoffSnapshot _RobotDevice::BeckhoffSnapshot() const
{
    return beckhoff::Beckhoff_Motor::GetInstance().Snapshot();
}

// ERCP
bool _RobotDevice::BeckhoffERCPOperateState(bool state) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().ERCPOperateState(state);
}
bool _RobotDevice::BeckhoffIsERCPOnline() const
{
    return beckhoff::Beckhoff_Motor::GetInstance().IsERCPOnline();
}
bool _RobotDevice::BeckhoffIsERCPReady() const
{
    return beckhoff::Beckhoff_Motor::GetInstance().IsERCPReady();
}

bool _RobotDevice::BeckhoffEmergencyStop(bool active) const
{
    return beckhoff::Beckhoff_Motor::GetInstance().EmergencyStop(active);
}

} // namespace ercp
