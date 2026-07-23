#pragma once
#include <boost/asio.hpp>
#include <boost/signals2.hpp>
#include <tbb/concurrent_map.h>
#include <Eigen/Dense>
#include <Poco/Net/UDPServer.h>
#include <Poco/Net/DatagramSocket.h>
using UdpClient = Poco::Net::DatagramSocket;
using UdpServer = Poco::Net::UDPServer;

#include "utils.h"
#include "robot_config.h"
#include "robot_udp_v2_runtime.hpp"
#include "yunsbot_config.h"

namespace task {
    template <typename... Args>
    class SequentialTasks;

    template <typename... Args>
    class _TasksBase;
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

            bool SwitchAutoMode(bool enable);
            bool SwitchLogger(bool enable);
            bool AddFLog(double dF1, double dF2, double dF3, double dL, double dAx1, double dAx2, double dAx3);
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
            std::atomic_bool m_RobotAutoMode = { false };
            std::atomic_bool m_RobotStarted = { false };
            std::atomic_bool m_RobotStarting = { false };
            std::atomic_bool m_RobotStopping = { false };

            std::shared_future<bool> m_future;

            mutable std::mutex m_report_mutex;
            tbb::concurrent_map<std::string, int> m_init_report;
            tbb::concurrent_map<std::string, int> m_deinit_report;
            std::shared_ptr<task::SequentialTasks<>> InitTasks;
            std::shared_ptr<task::SequentialTasks<>> DeinitTasks;

        private:
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

            robot_udp_v2::AppliedCommandTracker m_applied_commands;
            std::atomic<std::uint16_t> m_active_source{0};
            std::atomic<bool> m_command_fresh{false};
            std::atomic<std::uint64_t> m_accepted_command_received_unix_ns{0};
            std::atomic<std::uint64_t> m_lifecycle_changed_unix_ns{0};
            const std::uint64_t m_status_session_id;
            std::uint64_t m_status_sequence = 0;

            void StartControlThreads();
            void ExitControlThreads();
//            void ControlRunnable(double t);
            void ControlRunnable2(double t);

            void build_follow_cmd(
                const protocol::v2::ControlPayload &cmd, beckhoff_follow_cmd &follow_cmd);

            protocol::v2::AppliedCommandPayload AppliedCommands() const;
            protocol::v2::Source ActiveSource() const;
            std::uint64_t AcceptedCommandReceivedUnixNs() const;
            std::uint64_t LifecycleChangedUnixNs() const;
            protocol::v2::Bytes BuildStatusPacket();


            YunSBot &parent;
        } base;

        /// <summary>
        /// Master and Cloud V2 control/status channels.
        /// </summary>
        struct _control_channel : public Poco::Net::UDPHandler {

            friend class YunSBot;

        public:
            bool IsOnline(double overtime = 0.1) const;
            bool GetCommand(protocol::v2::ControlPayload &cmd,
                robot_udp_v2::CommandMetadata &metadata, double overtime = 0.1) const;
            bool LatestCommand(protocol::v2::ControlPayload &cmd,
                robot_udp_v2::CommandMetadata &metadata) const;
            robot_udp_v2::ReceiveStats Stats() const;
            int SendStatus(const protocol::v2::Bytes &data);

        protected:
            _control_channel(YunSBot &p, protocol::v2::Source source,
                std::string remote_address, std::uint16_t status_port,
                std::uint16_t control_port, bool loopback_only);
            _control_channel(const _control_channel &) = delete;

        private:
            YunSBot &parent;
            const protocol::v2::Source source;
            robot_udp_v2::CommandReceiver receiver;

            UdpClient client;
            std::shared_ptr<UdpServer> server;
            Poco::Net::UDPHandler::List handlers;
            std::chrono::steady_clock::time_point last_stats_log_{};

            void processData(char *buf) override;
            void processError(char *buf) override;
        } master, situaware;

        /// <summary>
        /// 机器人全局上下文
        /// </summary>
        struct _context {

            friend class YunSBot;

            Eigen::Vector3d following_old_target = Eigen::Vector3d::Zero();

            std::atomic<double> following_complement_master = { 0.0 };  //按键补偿
            std::atomic<double> following_complement_autofollow = { 0.0 };    //输送轮
            std::atomic<double> following_complement_force = { 0.0 };  //力控补偿

            std::atomic<bool> following_autofollow_enable = { false };
            std::atomic<double> following_autofollow_target = { 0.0 };

            std::atomic<double> cannula_bowing_pos;

            std::atomic<bool> base_enabled = { false };
            std::atomic<double> base_timer = { -1.0 };

            std::atomic<double> force_fb = { 0.0 };
            std::atomic<double> force_target_max = { 10.0 };
            std::atomic<double> force_target_min = { -10.0 };
            std::atomic<double> out_limit_max = { 1.0 };
            std::atomic<double> out_limit_min = { -1.0 };
            std::atomic<double> Kp = { 0.5 };
            std::atomic<double> Ki = { 0.2 };
            std::atomic<double> Kd = { 0.3 };
            std::atomic<double> erro = { 0.0 };
            std::atomic<double> erro_last = { 0.0 };
            std::atomic<double> erro_last_last = { 0.0 };
            std::atomic<double> pid_out = { 0.0 };

            std::atomic<bool> test_[9] = { false };

            std::atomic<double> init_pos_[9] = { 0.0 };

            std::atomic<double> tar_pos_[9] = { 0.0 };

            std::atomic<double> pas_enc_ofs = { 0.0 };

        
            std::atomic<double> fold_tar_pos_[3] = { 0.0 };
            std::atomic<double> open_tar_pos[13] = {30, 139.3304,122.9004,91.9986,0,41.9833, 138.6022, 113.0016, -37.8703,-73.2925,0,0,0 };
            std::atomic<double> fold_tar_pos[13] = {30, 195.3304,6.9004,  146.9986,0,41.9833, 200.4861, 14.9878,  -6.2073,-73.2925, 0,0,0 };
        protected:
            _context();
        } context;

    protected:
        YunSBot();
        YunSBot(const YunSBot &) = delete;
    };

} // namespace ercp
