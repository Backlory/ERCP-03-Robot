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

    void _RobotDevice::CreateExtraLogger(std::string name)
    {
        if (name.empty()) {
            name = fmt::format("{:.06f}.log", ilsr::Time::logtime());
        }
        auto log = std::make_shared<ilsr::Logger>(name, GetLogPath());
        std::lock_guard<decltype(m_logger_mutex)> lock(m_logger_mutex);
        m_extra_logger.swap(log);
    }

    std::shared_ptr<ilsr::Logger> _RobotDevice::GetExtraLogger()
    {
        std::lock_guard<decltype(m_logger_mutex)> lock(m_logger_mutex);
        return m_extra_logger;
    }

    void _RobotDevice::ReleaseExtraLogger()
    {
        std::lock_guard<decltype(m_logger_mutex)> lock(m_logger_mutex);
        m_extra_logger.reset();
    }

    ///////////////////////////////////////////////////////////////////////////

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

    _RobotDevice::~_RobotDevice()
    {
    }

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

    //bool _RobotDevice::BeckhoffRead() const {
    //    INT32 data[5];
    //    bool bIsRe = beckhoff::Beckhoff_Motor::GetInstance().Encoder(data);
    //    return true;// beckhoff::Beckhoff_Motor::GetInstance().ReadData();
    //}

    //bool _RobotDevice::BeckhoffWrite() const {
    //    return true;// beckhoff::Beckhoff_Motor::GetInstance().WriteData();
    //}

    bool _RobotDevice::BeckhoffMoveArmTo(bool bIsOpen) const {
        return beckhoff::Beckhoff_Motor::GetInstance().MoveArmTo(bIsOpen);
    }

    beckhoff_arm_move_state _RobotDevice::BeckhoffArmMoveState() const {
        return beckhoff::Beckhoff_Motor::GetInstance().MoveState();;
    }

    bool _RobotDevice::BeckhoffFollowData(unsigned long length, void* data) const {
        return beckhoff::Beckhoff_Motor::GetInstance().FollowOperationData(length, data);
    }

    std::uint32_t _RobotDevice::BeckhoffFollowDataResult(
        unsigned long length, const void *data) const
    {
        return beckhoff::Beckhoff_Motor::GetInstance().FollowOperationDataResult(length, data);
    }

    bool _RobotDevice::BeckhoffFollowData_Oneclick(double target, double bigAngle, double smlAngle) const {
        return true;
        //beckhoff::Beckhoff_Motor::GetInstance().FollowOperationData_Oneclick(target, bigAngle, smlAngle);
    }

    bool _RobotDevice::BeckhoffArmOperation(beckhoff_arm_operation iArmOper)const {
        return beckhoff::Beckhoff_Motor::GetInstance().ArmOperation(iArmOper);
    }

    //bool _RobotDevice::BeckhoffMotorMove(double data[13], int iType, int bIsUpdate) const {
    //    return beckhoff::Beckhoff_Motor::GetInstance().MotorMove(data, iType, bIsUpdate);
    //}

    bool _RobotDevice::BeckhoffLinearActuator(INT16 data[2])const {
        return beckhoff::Beckhoff_Motor::GetInstance().LinearActuator(data);
    }

    //bool _RobotDevice::BeckhoffMotorPara(int iReadPos, double& dActPos, double& dActVel, int& iActCurrent)const {
    //    return beckhoff::Beckhoff_Motor::GetInstance().MotorPara(iReadPos, dActPos, dActVel, iActCurrent);
    //}

    //bool _RobotDevice::BeckhoffMotorPara2(double dActPos[13], double dActVel[13], int iActCurrent[13])const {
    //    return beckhoff::Beckhoff_Motor::GetInstance().MotorPara2(dActPos, dActVel, iActCurrent);
    //}

    ////2-3-2
    //bool _RobotDevice::BeckhoffReadAsexPos(double asex_pos[21]) const
    //{
    //    return beckhoff::Beckhoff_Motor::GetInstance().ReadAsexPos(asex_pos);
    //}

    //2-3-3
    bool _RobotDevice::BeckhoffReadAsexPos(double asex_pos[19])const {
        return beckhoff::Beckhoff_Motor::GetInstance().ReadAsexPos(asex_pos);
    }

    bool _RobotDevice::BeckhoffEncoder(INT32 data[5])const {
        return beckhoff::Beckhoff_Motor::GetInstance().Encoder(data);
    }
    bool _RobotDevice::BeckhoffSensor(INT16 data[7])const {
        return beckhoff::Beckhoff_Motor::GetInstance().Sensor(data);
    }

    double _RobotDevice::BeckhoffFollowLength(void) const {
        return beckhoff::Beckhoff_Motor::GetInstance().Follow_Length();
    }

    double _RobotDevice::BeckhoffSmallWhell(void) const {
        return beckhoff::Beckhoff_Motor::GetInstance().SmallWhell();

    }

    double _RobotDevice::BeckhoffBigWhell(void) const {
        return beckhoff::Beckhoff_Motor::GetInstance().BigWhell();
    }

    double _RobotDevice::BeckhoffSmallWhellCalc(void) const {
        return beckhoff::Beckhoff_Motor::GetInstance().SmallWhellCalc();

    }

    double _RobotDevice::BeckhoffBigWhellCalc(void) const {
        return beckhoff::Beckhoff_Motor::GetInstance().BigWhellCalc();
    }


    double _RobotDevice::BeckhoffRotateDegree(void) const {
        return beckhoff::Beckhoff_Motor::GetInstance().RotateDegree();
    }

    bool _RobotDevice::Beckhoff_Switch(gpio_output_t out_switch) const {
        return beckhoff::Beckhoff_Motor::GetInstance().Output_Switch(out_switch);
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

    double _RobotDevice::BeckhoffLifter() const {
        return beckhoff::Beckhoff_Motor::GetInstance().Lifter();
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

    double _RobotDevice::BeckhoffGetERCPDeliverForce() const {
        return beckhoff::Beckhoff_Motor::GetInstance().GetERCPDeliverForce();
    }
    double _RobotDevice::BeckhoffGetERCPGuidwireForce() const {
        return beckhoff::Beckhoff_Motor::GetInstance().GetERCPGuidwireForce();
    }
    double _RobotDevice::BeckhoffGetERCPDeliverPos() const
    {
        return beckhoff::Beckhoff_Motor::GetInstance().GetERCPDeliverPos();
    }
} // namespace ercp
