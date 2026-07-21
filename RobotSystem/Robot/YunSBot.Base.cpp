#include <assert.h>
#include <vector>
#include <boost/make_shared.hpp>
#include "task.hpp"
#include "robot_settings.hpp"
#include "robot_devices.h"
#include "robot_config.h"
#include "YunSBot.h"
#include "RPC/ArmModule.hpp"

using namespace task;

namespace ercp {

    extern const double CUTTER_NEGLIM;
    extern const double CUTTER_POSLIM;
    extern const double CUTTER_SPEED_RATIO;

#define CMD_BACK     (11)
#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

    YunSBot::YunSBot()
        : base(*this)
        , visual(*this)
        , master(*this)
        , situaware(*this)
    {
        ROBOT_INFO(true, "Start lingcai robot !");
        base.StartThreads();

        rpc::ArmModule::GetInstance().Resume<rpc::ArmModule>();
    }

    YunSBot::_context::_context()
        : cannula_bowing_pos(CUTTER_POSLIM)
    {
    }

    ///////////////////////////////////////////////////////////////////////////

    YunSBot::_base::_base(YunSBot &p)
        : parent(p)
    {
        InitStartingTask();
        InitClosingTask();
        InitBackgroundTask();

        BeforeRobotStarting.connect([&]() { m_RobotAutoMode = false; });
        OnRobotStartSucceed.connect([&]() { StartControlThreads(); });
        BeforeRobotStopping.connect([&]() {
            m_RobotAutoMode = false;
            ExitControlThreads();
        });
        OnRobotStopFailed.connect([&]() { StartControlThreads(); });

        work = boost::make_shared<boost::asio::io_service::work>(this->io_service);
    }

    void YunSBot::_base::StartThreads()
    {
        m_bg_worker = boost::make_shared<boost::thread>([this]() {
            ROBOT_THREADNAME("BgAuto");

            while (!boost::this_thread::interruption_requested()) {
                ////////////////////////////////////////////////////////////////////////
                double t = ilsr::Time::wall_time();
                try {
                    OnBackground(t);
                } catch (std::exception &e) {
                    //ROBOT_ERROR(true, "[Background Error]: " << e.what())
                }
                double te = ilsr::Time::wall_time();
                ////////////////////////////////////////////////////////////////////////
                double sleep = std::max(0.00, 0.020 - (te - t)); // max 50Hz
                ilsr::Time::sleep_for(sleep);
                boost::this_thread::interruption_point();
            }
        });

        worker = boost::make_shared<std::thread>([this]() {
            try {
                boost::system::error_code ec;
                io_service.run(ec);
            } catch (std::exception e) {
                ROBOT_ERROR(true, "IO context: " << e.what())
            }
        });
    }

    void YunSBot::_base::StartControlThreads()
    {
        if (!m_ctrl_worker) {
            m_ctrl_worker = boost::make_shared<boost::thread>([this]() {
                ROBOT_THREADNAME("BgAuto");

                while (!boost::this_thread::interruption_requested()) {
                    ////////////////////////////////////////////////////////////////////////
                    double t = ilsr::Time::wall_time();
                    try {
                        OnControl(t);
                    } catch (std::exception &e) {
                        //ROBOT_ERROR(true, "[Control Error]: " << e.what())
                    }
                    double te = ilsr::Time::wall_time();
                    ////////////////////////////////////////////////////////////////////////
                    double sleep = std::max(0.00, 0.008 - (te - t)); // max 125Hz
                    ilsr::Time::sleep_for(sleep);
                    boost::this_thread::interruption_point();
                }
            });
        }
    }

    void YunSBot::_base::ExitControlThreads()
    {
        if (m_ctrl_worker) {
            m_ctrl_worker->interrupt();
            m_ctrl_worker->join();
            m_ctrl_worker.reset();
        }
    }

    YunSBot::_base::~_base()

    {
        work.reset();
        io_service.reset();

        // Stop worker
        if (m_bg_worker) {
            m_bg_worker->interrupt();
            m_bg_worker->join();
        }
        if (m_RobotStarted) {
            Stop();
            m_future.wait();
        }
    }

    bool YunSBot::_base::Start(size_t total)
    {
        if (m_future.valid()) {
            if (m_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
                return false;
            }
            m_future.get();
        }

        m_future = std::async(std::launch::async, [this, total]() {
            size_t times = total;
            if (IsRobotRunning()) {
                return false;
            }

            ROBOT_INFO(true, "Robot starting ...")
            m_RobotStarting = true;

            // test
            OnRobotStartSucceed();

            if (m_init_report.size() == 0) {
                InitTasks->report(m_init_report);
            }

            BeforeRobotStarting();

            while (times > 0) {
                ROBOT_INFO(true, fmt::format("Robot start try {} ...", total - times))
                if (InitTasks->run(m_init_report)) {
                    break;
                }

                auto err = InitTasks->get_error();
                try {
                    std::rethrow_exception(err);
                } catch (std::exception e) {
                    ROBOT_ERROR(true, "Error: " << e.what())
                }
                times--;
            }

            if (times <= 0) {
                ROBOT_INFO(true, "Robot start failed.")
                ROBOT_INFO(true, GetStartInfo());
                OnRobotStartFailed();
                m_RobotStarting = false;
                return false;
            }

            ROBOT_INFO(true, "Robot started.")
            ROBOT_INFO(true, GetStartInfo());
            m_RobotStarted = true;
            OnRobotStartSucceed();
            m_RobotStarting = false;

            m_deinit_report.clear();
            return true;
        });
        return true;
    }

    bool YunSBot::_base::Stop()
    {
        // 需等到机械臂折叠后才能关机
        auto &arm = rpc::ArmModule::GetInstance();
        if (arm.get_current_state() > rpc::arm_state_t::A3_Folded) {
            return false;
        }

        if (m_future.valid()) {
            if (m_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
                return false;
            }
            m_future.get();
        }

        m_future = std::async(std::launch::async, [this]() {
            if (!IsRobotRunning()) {
                return false;
            }

            ROBOT_INFO(true, "Robot stopping ...")
            m_RobotStopping = true;
            BeforeRobotStopping();

            // init_info_t info = m_init_info;
            if (m_deinit_report.size() == 0) {
                DeinitTasks->report(m_deinit_report);
            }

            if (!DeinitTasks->run(m_deinit_report)) {
                ROBOT_INFO(true, "Robot stop failed.")
                ROBOT_INFO(true, GetStopInfo());
                OnRobotStopFailed();
                OnRobotStopEnd();
                m_RobotStopping = false;
                return false;
            }

            auto err = InitTasks->get_error();
            try {
                std::rethrow_exception(err);
            } catch (std::exception e) {
                ROBOT_ERROR(true, "Error: " << e.what())
            }

            ROBOT_INFO(true, "Robot stopped.")
            ROBOT_INFO(true, GetStopInfo());
            m_RobotStarted = false;
            if (OnRobotStopped)
                OnRobotStopped();
            OnRobotStopEnd();
            m_RobotStopping = false;

            m_init_report.clear();
            return true;
        });
        return true;
    }

    std::vector<std::pair<std::string, int>> YunSBot::_base::GetStartReport() const
    {
        std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
        std::vector<std::pair<std::string, int>> map;
        for (auto p : m_init_report) {
            map.emplace_back(p);
        }
        return map;
    }

    std::vector<std::pair<std::string, int>> YunSBot::_base::GetStopReport() const
    {
        std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
        std::vector<std::pair<std::string, int>> map;
        for (auto p : m_deinit_report) {
            map.emplace_back(p);
        }
        return map;
    }

    std::vector<std::pair<std::wstring, int>> YunSBot::_base::GetStartReportW() const
    {
        std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
        std::vector<std::pair<std::wstring, int>> map;
        for (auto p : m_init_report) {
            map.emplace_back(ilsr::convert(p.first), p.second);
        }
        return map;
    }

    std::vector<std::pair<std::wstring, int>> YunSBot::_base::GetStopReportW() const
    {
        std::lock_guard<decltype(m_report_mutex)> lock(m_report_mutex);
        std::vector<std::pair<std::wstring, int>> map;
        for (auto p : m_deinit_report) {
            map.emplace_back(ilsr::convert(p.first), p.second);
        }
        return map;
    }

    std::string YunSBot::_base::GetStartInfo() const
    {
        std::string info = "Robot start report:\r\n";
        auto rep = GetStartReport();
        for (auto &r : rep) {
            info += fmt::format("{}, {}\r\n", r.first, r.second);
        }
        return std::move(info);
    }

    std::string YunSBot::_base::GetStopInfo() const
    {
        std::string info = "Robot stop report:\r\n";
        auto rep = GetStopReport();
        for (auto &r : rep) {
            info += fmt::format("{}, {}\r\n", r.first, r.second);
        }
        return std::move(info);
    }

    bool YunSBot::_base::IsAutoMode() const { return m_RobotAutoMode; }

    bool YunSBot::_base::IsRobotRunning() const { return m_RobotStarted; }

    bool YunSBot::_base::IsRobotStarting() const { return m_RobotStarting; }

    bool YunSBot::_base::IsRobotStopping() const { return m_RobotStopping; }

    bool YunSBot::_base::IsLogging() const
    {
        std::lock_guard<decltype(m_log_mutex)> _(m_log_mutex);
        return m_logger != nullptr;
    }

    boost::asio::io_service &YunSBot::_base::GetIOServer() { return io_service; }

    bool YunSBot::_base::SwitchAutoMode(bool enable)
    {
        bool b1 = IsRobotRunning();
        bool b2 = parent.visual.IsVisionOnline();
        /*std::cout << "b1:" << b1 << std::endl;
        std::cout << "b2:" << b2 << std::endl;*/
        if (enable && IsRobotRunning() && parent.visual.IsVisionOnline()) {
            m_RobotAutoMode = true;
        } else {
            m_RobotAutoMode = false;
        }
        return true;
    }

    bool YunSBot::_base::SwitchLogger(bool enable)
    {
        std::lock_guard<decltype(m_log_mutex)> _(m_log_mutex);
        if (enable) {
            if (!m_logger) {
                m_logger = std::make_shared<ilsr::Logger>(ilsr::Time::logtime() + ".csv", GetLogPath());
                m_logger->AddLog("t,bx,by,bz,br,x,y,z,r,vx,vy,vz,vr,sf,sq,cf,wf,hf\n");
            }
        } else {
            if (m_logger) {
                m_logger.reset();
            }
        }
        return m_logger != nullptr;
    }

    bool YunSBot::_base::AddFLog(double dF1, double dF2, double dF3, double dL, double dAx1, double dAx2, double dAx3) {
        if (m_logger) {
            char buf[1024];
            sprintf(buf, "'%s,%f,%f,%f,%f,%f,%f,%f\n", ilsr::Time::timestamp().c_str(), dL, dF1, dF2, dF3, dAx1, dAx2, dAx3);
            std::lock_guard<decltype(m_log_mutex)> _(m_log_mutex);

            m_logger->AddLog(buf);
        
        }
        return true;
    }

    bool YunSBot::_base::AddFRecord(double dFValue, double dPos) {
        if (!m_FRecord) {
            m_FRecord = std::make_shared<ilsr::Logger>("Force_Record.csv", "d:\\cyl\\record\\");
            m_FRecord->AddLog("\n\n");
        }

        m_FRecord->AddLog(std::to_string(dFValue) +"," + std::to_string(dPos)+  "\n");

        return true;
    }


    ///////////////////////////////////////////////////////////////////////////

    //extern task::seque_ptr<> MotorInitor(motor_t mid);
    //extern task::seque_ptr<> MotorDeInitor(motor_t mid);
    //extern task::seque_ptr<> GpioInitor(motor_t mid);

    //extern task::seque_ptr<> PusherInitor();
    //extern task::seque_ptr<> StepInitor();

    task::paral_ptr<> BaseInitor()
    {
        auto seq = std::make_shared<ParallelTasks<>>(u8"基座");
        // delete by cyl 原有电机调用废弃不用
        //seq->emplace(MotorInitor(motor_t::base_1));
        //seq->emplace(MotorInitor(motor_t::base_2));
        //seq->emplace(MotorInitor(motor_t::base_3));
        return seq;
    }

    task::paral_ptr<> BaseDeInitor()
    {
        auto seq = std::make_shared<ParallelTasks<>>(u8"基座");
        // delete by cyl 原有电机调用废弃不用
        //seq->emplace(MotorDeInitor(motor_t::base_1));
        //seq->emplace(MotorDeInitor(motor_t::base_2));
        //seq->emplace(MotorDeInitor(motor_t::base_3));
        return seq;
    }

    void YunSBot::_base::InitStartingTask()
    {
        InitTasks = std::make_shared<SequentialTasks<>>(u8"开机");
        using task = _TaskBase<>;

        if (GetSettings().Device.Module.Arm()) {
            InitTasks->emplace(
                u8"启动机械臂模块",
                []() {
                    auto &arm = rpc::ArmModule::GetInstance();
                    arm.Initialize();
                    sleep_ms(200);
                    rpc::ArmModule::state_t a = arm.get_current_state();
                    return arm.get_current_state() >=rpc::ArmModule::state_t::A2_Inited;
                },
                false, 1);

            InitTasks->emplace(
                u8"更新机械臂状态",
                []() {
                    auto& arm = rpc::ArmModule::GetInstance();
                    auto& robot = GetRobot();

                    if (beckhoff_arm_move_state::BAMS_OPENED == robot.BeckhoffArmMoveState()) {
                        // 展开完成状态
                        return arm.GotoState(rpc::arm_state_t::A4_Opened);
                    } else if (beckhoff_arm_move_state::BAMS_FOLDED == robot.BeckhoffArmMoveState()) {
                        // 折叠完成状态
                        return arm.GotoState(rpc::arm_state_t::A3_Folded);
                    } else if (beckhoff_arm_move_state::BAMS_FOLLOWING
                            == robot.BeckhoffArmMoveState()
                        || beckhoff_arm_move_state::BAMS_FOLLOWED == robot.BeckhoffArmMoveState()) {
                        return arm.GotoState(rpc::arm_state_t::A5_Following); // ← 添加这个
                    }
                    return true;
                },
                false, 1);
        }

        ROBOT_INFO(true, "Robot starting list:\n" << InitTasks->dump());
    }

    void YunSBot::_base::InitClosingTask()
    {
        DeinitTasks = std::make_shared<SequentialTasks<>>(u8"关机");

        // 注册关闭行为
        {
            auto paral = std::make_shared<ParallelTasks<>>(u8"关闭电机");

            if (GetSettings().Device.Module.Arm()) {
                DeinitTasks->emplace(
                    u8"关闭机械臂模块",
                    []() {
                        auto &arm = rpc::ArmModule::GetInstance();
                        return arm.DeInitialize();
                    },
                    true, 1);
            }

            DeinitTasks->emplace(paral);
        }

        //ROBOT_INFO(true, "Robot closing list:\n" << DeinitTasks->dump());
    }

    extern robot_status GetRobotStatus();
    extern std::string GetRobotStatusStr(robot_status &rstate);


    void YunSBot::_base::InitBackgroundTask()
    {
        OnBackground.connect([this](double t) {
            // 发送状态
            static double t0 = 0;
            if (t - t0 > 0.02) {
                auto rstate = GetRobotStatus();

                control_cmd cmd;
                if (m_RobotAutoMode) {
                    parent.visual.GetVisionCmd(cmd, 1.0);
                } else {
                    parent.master.GetMasterCmd(cmd, 0.1);
                }

                //将检查ID与cmd,robot状态指令相绑定输出日志
                loguru::g_stderr_verbosity = loguru::Verbosity_OFF;
                std::string examine_id = "pig";
                if ("" == examine_id) {
                    ROBOT_INFO(true, fmt::format("Examine ID is empty"));
                } else {
                    // cmd
                    ROBOT_INFO(true,
                        fmt::format("Examine ID: {},\
                            Bend_lr: {},\
                            Bend_ud: {},\
                            Move: {},\
                            Rotate: {},\
                            Cutter_move: {},\
                            Cutter_bend: {},\
                            Wire_feed: {},\
                            Cutter_rotate: {},\
                            X: {},\
                            Y: {},\
                            Z: {}",
                            examine_id, cmd.vel_bend_lr, cmd.vel_bend_ud, cmd.vel_move,
                            cmd.vel_rotate, cmd.cannula.vel_cutter_move,
                            cmd.cannula.vel_cutter_bend, cmd.cannula.vel_wire_feed,
                            cmd.cannula.vel_cutter_rotate, cmd.base.vel_x, cmd.base.vel_y,
                            cmd.base.vel_r));
                    // robot状态
                    ROBOT_INFO(true, GetRobotStatusStr(rstate));
                    loguru::g_stderr_verbosity = loguru::Verbosity_INFO;
                }

                // 指令反馈
                int i = 0, const off = 10;
                rstate._reserved[off + (i++)] = cmd.vel_bend_lr;
                rstate._reserved[off + (i++)] = cmd.vel_bend_ud;
                rstate._reserved[off + (i++)] = cmd.vel_move;
                rstate._reserved[off + (i++)] = cmd.vel_rotate;
                rstate._reserved[off + (i++)] = cmd.cannula.vel_cutter_move;
                rstate._reserved[off + (i++)] = cmd.cannula.vel_cutter_bend;
                rstate._reserved[off + (i++)] = cmd.cannula.vel_wire_feed;
                rstate._reserved[off + (i++)] = cmd.cannula.vel_cutter_rotate;
                rstate._reserved[off + (i++)] = cmd.base.vel_x;
                rstate._reserved[off + (i++)] = cmd.base.vel_y;
                rstate._reserved[off + (i++)] = cmd.base.vel_r;

                // Serialize
                static std::vector<char> buffer(sizeof(rstate));
                std::copy((char *)&rstate, (char *)&rstate + buffer.size(), buffer.data());
                parent.master.SendStatus(buffer);
                parent.situaware.SendStatus(buffer);      // 情景感知发送流
                t0 = t;
            }
        });

        // 主从控制线程
        OnControl.connect(
            boost::bind(&YunSBot::_base::ControlRunnable2, this, boost::placeholders::_1));
    }

} // namespace ercp
