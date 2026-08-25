#pragma once
#include <cstddef>
#include <future>
#include <mutex>
#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <tbb/concurrent_map.h>
#include <Eigen/Dense>
#include <Poco/Net/UDPServer.h>
#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/IPAddress.h>
using UdpClient = Poco::Net::DatagramSocket;
using UdpServer = Poco::Net::UDPServer;

#include "utils.h"
#include "robot_config.h"
#include "robot_udp_v3_runtime.hpp"
#include "yunsbot_config.h"

namespace task {
template <typename... Args> class SequentialTasks;

template <typename... Args> class _TasksBase;
} // namespace task

namespace ercp {

class YunSBot {
public:
    static YunSBot &GetInstance()
    {
        static YunSBot lc;
        return lc;
    }

public:
    /// <summary>
    /// 启停控制，主从切换
    /// </summary>
    struct _base {

        friend class YunSBot;

    public:
        /// 开机
        bool Start(size_t times = 5);
        /// 关机
        bool Stop();

        bool IsAutoMode() const;
        bool IsRobotRunning() const;
        bool IsRobotStarting() const;
        bool IsRobotStopping() const;
        bool IsLogging() const;

        // Emergency stop requests are merged inside Robot. The Master UDP
        // request and the legacy HTTP/RPC request must not clear each other.
        bool SetMasterUdpEmergencyStop(bool active);
        bool SetRpcEmergencyStop(bool active);
        bool EmergencyStopActive() const;

        bool SwitchAutoMode(bool enable);
        bool SwitchLogger(bool enable);
        bool AddFLog(double dF1,
                     double dF2,
                     double dF3,
                     double dL,
                     double dAx1,
                     double dAx2,
                     double dAx3);
        bool AddFRecord(double dFValue, double dPos);

        std::string GetStartInfo() const;
        std::string GetStopInfo() const;
        boost::shared_ptr<boost::thread> GetWorker() { return m_bg_worker; }

        std::vector<std::pair<std::wstring, int>> GetStartReportW() const;
        std::vector<std::pair<std::wstring, int>> GetStopReportW() const;
        std::vector<std::pair<std::string, int>> GetStartReport() const;
        std::vector<std::pair<std::string, int>> GetStopReport() const;

        boost::asio::io_service &GetIOServer();

    public:
        boost::signals2::signal<void(void)> BeforeRobotStarting;
        boost::signals2::signal<void(void)> OnRobotStartFailed;
        boost::signals2::signal<void(void)> OnRobotStartSucceed;

        boost::signals2::signal<void(void)> BeforeRobotStopping;
        boost::signals2::signal<void(void)> OnRobotStopFailed;
        boost::signals2::signal<void(void)> OnRobotStopEnd;
        boost::function<void(void)> OnRobotStopped;

    protected:
        _base(YunSBot &p);
        _base(const _base &) = delete;
        ~_base();

    protected:
        /// 设备启动任务
        void InitStartingTask();
        /// 设备关闭任务
        void InitClosingTask();
        /// 后台任务
        void InitBackgroundTask();

        void StartThreads();

    protected:
        std::atomic_bool m_RobotAutoMode = {false};
        std::atomic_bool m_RobotStarted = {false};
        std::atomic_bool m_RobotStarting = {false};
        std::atomic_bool m_RobotStopping = {false};

        std::shared_future<bool> m_future;

        mutable std::mutex m_report_mutex;
        tbb::concurrent_map<std::string, int> m_init_report;
        tbb::concurrent_map<std::string, int> m_deinit_report;
        std::shared_ptr<task::SequentialTasks<>> InitTasks;
        std::shared_ptr<task::SequentialTasks<>> DeinitTasks;

    private:
        bool WaitForLifecycleTask();
        bool ExecuteStart(std::size_t totalAttempts);
        bool RunStartTasks(std::size_t totalAttempts);
        bool ExecuteStop();
        void LogTaskError(const std::exception_ptr &error) const;
        void RunPeriodicLoop(const char *threadName,
                             double intervalSeconds,
                             boost::signals2::signal<void(double)> &callback);

        boost::shared_ptr<boost::thread> m_bg_worker = nullptr;
        boost::signals2::signal<void(double)> OnBackground;

        boost::shared_ptr<boost::thread> m_ctrl_worker = nullptr;
        boost::signals2::signal<void(double)> OnControl;

        boost::asio::io_service io_service;
        boost::shared_ptr<boost::asio::io_service::work> work;
        boost::shared_ptr<std::thread> worker;

        mutable std::mutex m_log_mutex;
        std::shared_ptr<ilsr::Logger> m_logger;
        std::shared_ptr<ilsr::Logger> m_FRecord; // 测试记录力反馈电压值

        robot_udp_v3::AppliedCommandTracker m_applied_commands;
        // Task 11: Master 强制优先仲裁开关(默认启用,构造时从 basic.master_priority 读)
        // 与 Cloud 命令因仲裁被丢弃的累计计数(联调统计)。
        std::atomic<bool> m_master_priority{true};
        std::atomic<std::uint64_t> m_master_priority_overrides{0};
        std::atomic<std::uint16_t> m_active_source{0};
        std::atomic<bool> m_command_fresh{false};
        std::atomic<std::uint64_t> m_accepted_command_received_unix_ns{0};
        std::atomic<std::uint64_t> m_lifecycle_changed_unix_ns{0};
        mutable std::mutex m_emergency_stop_mutex;
        bool m_master_udp_emergency_stop = false;
        bool m_rpc_emergency_stop = false;
        bool m_last_written_emergency_stop = false;
        bool m_has_written_emergency_stop = false;
        const std::uint64_t m_status_session_id;
        std::uint64_t m_status_sequence = 0;
        std::uint64_t m_last_sent_common_sample_unix_ns = 0;

        void StartControlThreads();
        void ExitControlThreads();
        //            void ControlRunnable(double t);
        void ControlRunnable2(double t);
        bool ApplyEmergencyStopLocked();

        protocol::v3::AppliedCommandPayload AppliedCommands() const;
        protocol::v3::Source ActiveSource() const;
        std::uint64_t AcceptedCommandReceivedUnixNs() const;
        std::uint64_t LifecycleChangedUnixNs() const;
        protocol::v3::Bytes BuildStatusPacket();

        YunSBot &parent;
    } base;

    /// <summary>
    /// Master and Cloud V3 control/status channels.
    /// </summary>
    struct _control_channel : public Poco::Net::UDPHandler {

        friend class YunSBot;

    public:
        bool IsOnline(double overtime = 0.1) const;
        bool GetCommand(protocol::v3::ControlPayload &cmd,
                        robot_udp_v3::CommandMetadata &metadata,
                        double overtime = 0.1) const;
        bool LatestCommand(protocol::v3::ControlPayload &cmd,
                           robot_udp_v3::CommandMetadata &metadata) const;
        robot_udp_v3::ReceiveStats Stats() const;
        int SendStatus(const protocol::v3::Bytes &data);

    protected:
        _control_channel(YunSBot &p,
                         protocol::v3::Source source,
                         std::string remote_address,
                         std::uint16_t status_port,
                         std::uint16_t control_port,
                         bool loopback_only);
        _control_channel(const _control_channel &) = delete;

    private:
        YunSBot &parent;
        const protocol::v3::Source source;
        robot_udp_v3::CommandReceiver receiver;

        UdpClient client;
        std::shared_ptr<UdpServer> server;
        Poco::Net::UDPHandler::List handlers;
        std::chrono::steady_clock::time_point last_stats_log_{};

        // M4: non-loopback channels only accept datagrams whose source IP
        // matches the configured remote peer (basic.master). Deployment
        // precondition: the Master's actual egress IP must equal the
        // configured address (NAT or other rewriting setups unsupported).
        bool filter_peer_ = false;
        Poco::Net::IPAddress expected_peer_;
        std::chrono::steady_clock::time_point last_peer_drop_log_{};

        void processData(char *buf) override;
        void processError(char *buf) override;
    } master, situaware;

protected:
    YunSBot();
    YunSBot(const YunSBot &) = delete;
};

} // namespace ercp
