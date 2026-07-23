#include <boost/filesystem.hpp>
#include "robot_device_inner.hpp"

namespace fs = boost::filesystem;

namespace ercp {

    ROBOT_API_MEMBER RobotDevice &GetRobot() { return _RobotDevice::GetInstance(); }

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

    void _RobotDevice::InitDevices(int verbose)
    {
        auto &settings = GetSettings();

        // 倍福驱动初始化
        {
            // 反复保证链接
            bool bIsBeckhoffRun = false;

            for ( int i = 0; i < 5; i++ ) {
                if (bIsBeckhoffRun = beckhoff::Beckhoff_Motor::GetInstance().OpenConn(
                    settings.Device.Beckhoff.Addr(), settings.Device.Beckhoff.Port()))
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

    bool _RobotDevice::BeckhoffMoveArmTo(bool bIsOpen) const {
        return beckhoff::Beckhoff_Motor::GetInstance().MoveArmTo(bIsOpen);
    }

    beckhoff_arm_move_state _RobotDevice::BeckhoffArmMoveState() const {
        return beckhoff::Beckhoff_Motor::GetInstance().MoveState();;
    }

    std::uint32_t _RobotDevice::BeckhoffFollowDataResult(
        unsigned long length, const void *data) const
    {
        return beckhoff::Beckhoff_Motor::GetInstance().FollowOperationDataResult(length, data);
    }

    bool _RobotDevice::BeckhoffArmOperation(beckhoff_arm_operation iArmOper)const {
        return beckhoff::Beckhoff_Motor::GetInstance().ArmOperation(iArmOper);
    }

    bool _RobotDevice::BeckhoffReadAsexPos(double asex_pos[19])const {
        return beckhoff::Beckhoff_Motor::GetInstance().ReadAsexPos(asex_pos);
    }

    double _RobotDevice::BeckhoffForce(INT16 iPos) const {
        return beckhoff::Beckhoff_Motor::GetInstance().Force(iPos);
    }

    bool _RobotDevice::IsOpen() const {
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
    bool _RobotDevice::BeckhoffIsERCPReady() const {
        return beckhoff::Beckhoff_Motor::GetInstance().IsERCPReady();
    }

} // namespace ercp
