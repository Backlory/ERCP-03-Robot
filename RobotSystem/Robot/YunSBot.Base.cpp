#include <assert.h>
#include <cmath>
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

#define CMD_BACK     (11)
#define sleep_ms(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

    YunSBot::YunSBot()
        : base(*this)
        , master(*this, protocol::v2::Source::Master, GetSettings().Basic.Master(),
              31001, 31002, false)
        , situaware(*this, protocol::v2::Source::Cloud, "127.0.0.1", 31003, 31004, true)
    {
        ROBOT_INFO(true, "Start lingcai robot !");
        base.StartThreads();

        rpc::ArmModule::GetInstance().Resume<rpc::ArmModule>();
    }

    YunSBot::_base::_base(YunSBot &p)
        : m_status_session_id(robot_udp_v2::MakeSessionId())
        , parent(p)
    {
        m_lifecycle_changed_unix_ns = robot_udp_v2::UnixNowNs();
        InitStartingTask();
        InitClosingTask();
        InitBackgroundTask();

        BeforeRobotStarting.connect([&]() {
            m_RobotAutoMode = false;
            m_active_source = 0;
            m_command_fresh = false;
            m_accepted_command_received_unix_ns = 0;
            m_lifecycle_changed_unix_ns = robot_udp_v2::UnixNowNs();
        });
        OnRobotStartSucceed.connect([&]() {
            m_lifecycle_changed_unix_ns = robot_udp_v2::UnixNowNs();
            StartControlThreads();
        });
        OnRobotStartFailed.connect([&]() {
            m_lifecycle_changed_unix_ns = robot_udp_v2::UnixNowNs();
        });
        BeforeRobotStopping.connect([&]() {
            const bool automaticMode = m_RobotAutoMode.load();
            const auto selectedSource = robot_udp_v2::SelectedControlSource(automaticMode);
            m_RobotAutoMode = false;
            ExitControlThreads();

            // The control loop is gone, so explicitly make the final PLC write a safety zero.
            // This uses the unchanged Beckhoff native 10-double + 6-BOOL command layout.
            protocol::v2::ControlPayload ignored;
            robot_udp_v2::CommandMetadata metadata;
            auto &channel = automaticMode ? parent.situaware : parent.master;
            if (!channel.LatestCommand(ignored, metadata)) {
                metadata.source = selectedSource;
            }
            const auto zero = robot_udp_v2::ZeroControl();
            beckhoff_follow_cmd followCommand;
            build_follow_cmd(zero, followCommand);
            const auto appliedAt = robot_udp_v2::UnixNowNs();
            const auto adsError
                = GetRobot().BeckhoffFollowDataResult(sizeof(followCommand), &followCommand);
            const bool succeeded = adsError == 0;
            m_applied_commands.MarkAttempt(zero, metadata,
                succeeded ? protocol::v2::ApplyResult::TimedOutToZero
                          : protocol::v2::ApplyResult::Failed,
                adsError, appliedAt, succeeded);

            m_active_source = 0;
            m_command_fresh = false;
            m_accepted_command_received_unix_ns = 0;
            m_lifecycle_changed_unix_ns = robot_udp_v2::UnixNowNs();
        });
        OnRobotStopFailed.connect([&]() { StartControlThreads(); });
        OnRobotStopEnd.connect([&]() {
            m_lifecycle_changed_unix_ns = robot_udp_v2::UnixNowNs();
        });

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
        const bool autoSourceOnline = parent.situaware.IsOnline(0.1);
        if (enable && IsRobotRunning() && autoSourceOnline) {
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

    void YunSBot::_base::InitStartingTask()
    {
        InitTasks = std::make_shared<SequentialTasks<>>(u8"开机");
        using task = _TaskBase<>;

        InitTasks->emplace(
            u8"启动机械臂模块",
            []() {
                auto &arm = rpc::ArmModule::GetInstance();
                arm.Initialize();
                sleep_ms(200);
                return arm.get_current_state() >= rpc::ArmModule::state_t::A2_Inited;
            },
            false, 1);

        InitTasks->emplace(
            u8"更新机械臂状态",
            []() {
                auto &arm = rpc::ArmModule::GetInstance();
                auto &robot = GetRobot();

                if (beckhoff_arm_move_state::BAMS_OPENED == robot.BeckhoffArmMoveState()) {
                    return arm.GotoState(rpc::arm_state_t::A4_Opened);
                } else if (beckhoff_arm_move_state::BAMS_FOLDED
                    == robot.BeckhoffArmMoveState()) {
                    return arm.GotoState(rpc::arm_state_t::A3_Folded);
                } else if (beckhoff_arm_move_state::BAMS_FOLLOWING
                        == robot.BeckhoffArmMoveState()
                    || beckhoff_arm_move_state::BAMS_FOLLOWED
                        == robot.BeckhoffArmMoveState()) {
                    return arm.GotoState(rpc::arm_state_t::A5_Following);
                }
                return true;
            },
            false, 1);

        ROBOT_INFO(true, "Robot starting list:\n" << InitTasks->dump());
    }

    void YunSBot::_base::InitClosingTask()
    {
        DeinitTasks = std::make_shared<SequentialTasks<>>(u8"关机");

        // 注册关闭行为
        {
            auto paral = std::make_shared<ParallelTasks<>>(u8"关闭电机");

            DeinitTasks->emplace(
                u8"关闭机械臂模块",
                []() {
                    auto &arm = rpc::ArmModule::GetInstance();
                    return arm.DeInitialize();
                },
                true, 1);

            DeinitTasks->emplace(paral);
        }

        //ROBOT_INFO(true, "Robot closing list:\n" << DeinitTasks->dump());
    }

    void YunSBot::_base::InitBackgroundTask()
    {
        OnBackground.connect([this](double t) {
            static double t0 = 0;
            if (t - t0 > 0.02) {
                const auto packet = BuildStatusPacket();
                if (!packet.empty()) {
                    parent.master.SendStatus(packet);
                    parent.situaware.SendStatus(packet);
                }
                t0 = t;
            }
        });

        // 主从控制线程
        OnControl.connect(
            boost::bind(&YunSBot::_base::ControlRunnable2, this, boost::placeholders::_1));
    }

    protocol::v2::AppliedCommandPayload YunSBot::_base::AppliedCommands() const
    {
        return m_applied_commands.Snapshot();
    }

    protocol::v2::Source YunSBot::_base::ActiveSource() const
    {
        return static_cast<protocol::v2::Source>(m_active_source.load());
    }

    std::uint64_t YunSBot::_base::AcceptedCommandReceivedUnixNs() const
    {
        return m_accepted_command_received_unix_ns.load();
    }

    std::uint64_t YunSBot::_base::LifecycleChangedUnixNs() const
    {
        return m_lifecycle_changed_unix_ns.load();
    }

    protocol::v2::Bytes YunSBot::_base::BuildStatusPacket()
    {
        const auto snapshot = GetRobot().BeckhoffSnapshot();
        const auto now = robot_udp_v2::UnixNowNs();
        protocol::v2::FullStatusPayload status;

        if (IsRobotStopping()) status.runtime.lifecycle = protocol::v2::RobotLifecycle::Stopping;
        else if (IsRobotStarting()) status.runtime.lifecycle = protocol::v2::RobotLifecycle::Starting;
        else if (IsRobotRunning()) status.runtime.lifecycle = protocol::v2::RobotLifecycle::Running;
        else status.runtime.lifecycle = protocol::v2::RobotLifecycle::Stopped;
        status.runtime.mode = IsAutoMode()
            ? protocol::v2::RobotMode::Automatic : protocol::v2::RobotMode::Manual;
        status.runtime.active_source = ActiveSource();
        status.runtime.flags = (snapshot.connection_state
                    != device::beckhoff::SnapshotConnectionState::Disconnected ? 1u << 0 : 0u)
            | (IsLogging() ? 1u << 1 : 0u)
            | (m_command_fresh.load() ? 1u << 2 : 0u);
        status.runtime.lifecycle_changed_unix_ns = LifecycleChangedUnixNs();
        status.runtime.accepted_command_received_unix_ns = AcceptedCommandReceivedUnixNs();

        status.beckhoff_common.move_state =
            static_cast<protocol::v2::BeckhoffMoveState>(snapshot.move_state);
        status.beckhoff_common.output_switches = snapshot.output_switches;
        status.beckhoff_common.power_level = snapshot.power_level;
        status.beckhoff_common.values = snapshot.common_values;
        status.raw_io.encoders = snapshot.encoders;
        status.raw_io.sensors = snapshot.sensors;
        status.ercp_state.flags = snapshot.ercp_flags;
        status.ercp_state.drive_errors = snapshot.ercp_drive_errors;
        status.ercp_state.motor_errors = snapshot.ercp_motor_errors;
        status.ercp_state.type = static_cast<protocol::v2::ErcpDeviceType>(snapshot.ercp_type);
        status.ercp_state.move_status =
            static_cast<protocol::v2::ErcpMoveState>(snapshot.ercp_move_status);
        status.ercp_feedback.values = snapshot.ercp_feedback;
        status.ercp_feedback.inject_state_01 =
            static_cast<protocol::v2::InjectorState>(snapshot.inject_state_01);
        status.ercp_feedback.inject_state_02 =
            static_cast<protocol::v2::InjectorState>(snapshot.inject_state_02);
        const auto applied_commands = AppliedCommands();
        status.applied_command = applied_commands;

        status.ads_diagnostics.snapshot_sequence = snapshot.sequence;
        status.ads_diagnostics.poll_started_unix_ns = snapshot.poll_started_unix_ns;
        status.ads_diagnostics.poll_completed_unix_ns = snapshot.poll_completed_unix_ns;
        status.ads_diagnostics.snapshot_published_unix_ns = snapshot.published_unix_ns;
        status.ads_diagnostics.connection_state
            = static_cast<protocol::v2::AdsConnectionState>(snapshot.connection_state);
        status.ads_diagnostics.valid_groups = snapshot.valid_groups;
        status.ads_diagnostics.stale_groups = snapshot.stale_groups;
        status.ads_diagnostics.consecutive_failed_polls = snapshot.consecutive_failed_polls;
        status.ads_diagnostics.overall_ads_error = snapshot.overall_ads_error;
        status.ads_diagnostics.common_ads_error = snapshot.common_ads_error;
        status.ads_diagnostics.raw_io_ads_error = snapshot.raw_io_ads_error;
        status.ads_diagnostics.ercp_state_ads_error = snapshot.ercp_state_ads_error;
        status.ads_diagnostics.ercp_feedback_ads_error = snapshot.ercp_feedback_ads_error;
        status.ads_diagnostics.command_write_ads_error
            = applied_commands.latest_write_attempt.ads_error;

        for (double &value : status.beckhoff_common.values) {
            if (!std::isfinite(value)) {
                value = 0;
                status.ads_diagnostics.stale_groups |= device::beckhoff::SnapshotCommon;
            }
        }
        for (double &value : status.ercp_feedback.values) {
            if (!std::isfinite(value)) {
                value = 0;
                status.ads_diagnostics.stale_groups |= device::beckhoff::SnapshotErcpFeedback;
            }
        }

        status.sampled_at_unix_ns = { now, snapshot.sampled_at_unix_ns[0],
            snapshot.sampled_at_unix_ns[1], snapshot.sampled_at_unix_ns[2],
            snapshot.sampled_at_unix_ns[3], now, snapshot.published_unix_ns, now };

        protocol::v2::Header header;
        header.message_type = protocol::v2::MessageType::RobotStatus;
        header.source = protocol::v2::Source::Robot;
        header.session_id = m_status_session_id;
        header.sequence = m_status_sequence++;
        header.sent_at_unix_ns = now;

        protocol::v2::Bytes packet;
        std::string error;
        if (!protocol::v2::encodeFullStatus(header, status, packet, &error)) {
            ROBOT_ERROR(GetSettings().Basic.Verbose() > 0,
                fmt::format("Robot V2 status encode failed: {}", error))
            packet.clear();
        }
        return packet;
    }

} // namespace ercp

