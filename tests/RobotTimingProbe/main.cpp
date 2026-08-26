#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "fake_ads_server.hpp"
#include "protocol/robot_udp_v3.hpp"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "Robot/YunSBot.h"

namespace protocol = ercp::protocol::v3;
namespace timing_probe = ercp::timing_probe;

namespace {

constexpr std::uint16_t kStatusPort = 31001;
constexpr std::uint16_t kControlPort = 31002;

std::uint64_t UnixNowNs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

std::uint64_t SteadyNowNs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

struct Options {
    int duration_ms = 3000;
    int frequency_hz = 125;
    int ads_delay_ms = 0;
    timing_probe::AdsDelayScope ads_delay_scope = timing_probe::AdsDelayScope::None;
    std::string wheel = "both";
    std::string pattern = "step";
    double value = 0.25;
    double max_follow_period_ms = 0;
    double max_applied_age_ms = 0;
    std::string csv_path;
};

const char *DelayScopeName(timing_probe::AdsDelayScope scope)
{
    switch (scope) {
    case timing_probe::AdsDelayScope::None:
        return "none";
    case timing_probe::AdsDelayScope::All:
        return "all";
    case timing_probe::AdsDelayScope::Reads:
        return "reads";
    case timing_probe::AdsDelayScope::Writes:
        return "writes";
    }
    return "unknown";
}

bool ParsePositiveInt(const char *text, int &value)
{
    try {
        const int parsed = std::stoi(text);
        if (parsed <= 0)
            return false;
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseNonNegativeInt(const char *text, int &value)
{
    try {
        const int parsed = std::stoi(text);
        if (parsed < 0)
            return false;
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseNonNegativeDouble(const char *text, double &value)
{
    try {
        const double parsed = std::stod(text);
        if (!std::isfinite(parsed) || parsed < 0)
            return false;
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseDelayScope(const std::string &text, timing_probe::AdsDelayScope &scope)
{
    if (text == "none") {
        scope = timing_probe::AdsDelayScope::None;
        return true;
    }
    if (text == "all") {
        scope = timing_probe::AdsDelayScope::All;
        return true;
    }
    if (text == "reads") {
        scope = timing_probe::AdsDelayScope::Reads;
        return true;
    }
    if (text == "writes") {
        scope = timing_probe::AdsDelayScope::Writes;
        return true;
    }
    return false;
}

void PrintUsage()
{
    std::cout
        << "RobotTimingProbe.exe [options]\n"
        << "  --duration-ms N             probe duration (default 3000)\n"
        << "  --frequency-hz N            fake Master rate (default 125)\n"
        << "  --ads-delay-ms N            fake ADS response delay (default 0)\n"
        << "  --ads-delay-scope S         none|all|reads|writes (default none)\n"
        << "  --wheel W                   lr|ud|both (default both)\n"
        << "  --pattern P                 constant|step (default step)\n"
        << "  --value V                   wheel command magnitude, -1..1 (default 0.25)\n"
        << "  --max-follow-period-ms N    fail if p95 follow-write interval is above N\n"
        << "  --max-applied-age-ms N      fail if p95 command age is above N\n"
        << "  --csv PATH                  write raw observations to PATH\n"
        << "\n"
        << "This tool starts RobotSystem code with an in-process fake ADS server.\n"
        << "It never connects to TwinCAT and never starts the real Master.\n";
}

bool ParseOptions(int argc, char **argv, Options &options)
{
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--help" || arg == "-h") {
            PrintUsage();
            return false;
        }
        if (index + 1 >= argc)
            return false;
        const char *value = argv[++index];

        if (arg == "--duration-ms") {
            if (!ParsePositiveInt(value, options.duration_ms))
                return false;
        } else if (arg == "--frequency-hz") {
            if (!ParsePositiveInt(value, options.frequency_hz) || options.frequency_hz > 500)
                return false;
        } else if (arg == "--ads-delay-ms") {
            if (!ParseNonNegativeInt(value, options.ads_delay_ms) || options.ads_delay_ms > 500)
                return false;
        } else if (arg == "--ads-delay-scope") {
            if (!ParseDelayScope(value, options.ads_delay_scope))
                return false;
        } else if (arg == "--wheel") {
            options.wheel = value;
            if (options.wheel != "lr" && options.wheel != "ud" && options.wheel != "both")
                return false;
        } else if (arg == "--pattern") {
            options.pattern = value;
            if (options.pattern != "constant" && options.pattern != "step")
                return false;
        } else if (arg == "--value") {
            try {
                options.value = std::stod(value);
            } catch (...) {
                return false;
            }
            if (!std::isfinite(options.value) || std::abs(options.value) > 1.0)
                return false;
        } else if (arg == "--max-follow-period-ms") {
            if (!ParseNonNegativeDouble(value, options.max_follow_period_ms))
                return false;
        } else if (arg == "--max-applied-age-ms") {
            if (!ParseNonNegativeDouble(value, options.max_applied_age_ms))
                return false;
        } else if (arg == "--csv") {
            options.csv_path = value;
            if (options.csv_path.empty())
                return false;
        } else {
            return false;
        }
    }
    return true;
}

struct StatusObservation {
    std::uint64_t received_steady_ns = 0;
    protocol::Header header;
    protocol::FullStatusPayload payload;
};

/**
 * @brief 接收 RobotSystem 发往 Master 的状态包并保存经过协议校验的样本。
 * @details 只绑定本地 31001，不修改 RobotSystem 状态；样本中的应用命令审计和 ADS
 *          诊断字段用于计算“命令进入—控制应用”的时间关系。
 */
class StatusCollector {
public:
    ~StatusCollector()
    {
        Stop();
    }

    bool Start(std::uint16_t port, std::string *error)
    {
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == INVALID_SOCKET) {
            if (error != nullptr)
                *error = "status socket failed";
            return false;
        }
        BOOL reuse = TRUE;
        setsockopt(socket_,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   reinterpret_cast<const char *>(&reuse),
                   sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(port);
        if (bind(socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) ==
            SOCKET_ERROR) {
            if (error != nullptr)
                *error = "status bind failed";
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
            return false;
        }
        running_.store(true, std::memory_order_release);
        worker_ = std::thread([this]() { ReceiveLoop(); });
        return true;
    }

    void Stop()
    {
        running_.store(false, std::memory_order_release);
        // ReceiveLoop uses a bounded 100 ms select timeout. Let it observe
        // running_=false before closing the descriptor; closing it first can
        // leave a Windows select/recv teardown in an undefined state.
        if (worker_.joinable())
            worker_.join();
        if (socket_ != INVALID_SOCKET) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
        }
    }

    std::vector<StatusObservation> Records() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return records_;
    }

private:
    void ReceiveLoop()
    {
        std::array<std::uint8_t, protocol::kMaxPacketSize> buffer{};
        while (running_.load(std::memory_order_acquire)) {
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(socket_, &read_set);
            timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;
            const int ready = select(0, &read_set, nullptr, nullptr, &timeout);
            if (ready <= 0)
                continue;

            sockaddr_in sender{};
            int sender_size = sizeof(sender);
            const int received = recvfrom(socket_,
                                          reinterpret_cast<char *>(buffer.data()),
                                          static_cast<int>(buffer.size()),
                                          0,
                                          reinterpret_cast<sockaddr *>(&sender),
                                          &sender_size);
            if (received <= 0)
                continue;

            protocol::Header header;
            protocol::FullStatusPayload payload;
            std::string error;
            if (!protocol::decodeFullStatus(buffer.data(),
                                            static_cast<std::size_t>(received),
                                            header,
                                            payload,
                                            &error))
                continue;
            if (header.source != protocol::Source::Robot)
                continue;
            std::lock_guard<std::mutex> lock(mutex_);
            records_.push_back({SteadyNowNs(), header, payload});
        }
    }

    SOCKET socket_ = INVALID_SOCKET;
    std::atomic<bool> running_{false};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::vector<StatusObservation> records_;
};

struct MasterObservation {
    std::uint64_t sent_steady_ns = 0;
    std::uint64_t sent_unix_ns = 0;
    std::uint64_t sequence = 0;
    bool sent = false;
};

double WheelValue(const Options &options, std::uint64_t sequence, std::uint64_t started_ns)
{
    if (options.pattern == "constant")
        return options.value;
    const auto elapsed_ms = (SteadyNowNs() - started_ns) / 1000000ull;
    return (elapsed_ms / 250ull) % 2 == 0 ? options.value : -options.value;
}

/**
 * @brief 以固定频率伪造 Master，仅发送 Robot UDP V3.1 控制包。
 * @details 发送端不依赖 Master 工程；大小/小拨轮分别映射到协议的
 *          `ScopeBendLr`/`ScopeBendUd`，每个包都有连续 session/sequence。
 */
void RunFakeMaster(const Options &options,
                   std::atomic<bool> &stop,
                   std::vector<MasterObservation> &observations,
                   std::mutex &observations_mutex)
{
    const SOCKET socket_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_handle == INVALID_SOCKET)
        return;

    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    target.sin_port = htons(kControlPort);

    const auto started = SteadyNowNs();
    const auto period = std::chrono::nanoseconds(1000000000ll / options.frequency_hz);
    auto next = std::chrono::steady_clock::now();
    std::uint64_t sequence = 1;
    constexpr std::uint64_t session = 0x54494D494E475052ull; // "TIMINGPR"

    while (!stop.load(std::memory_order_acquire)) {
        std::this_thread::sleep_until(next);
        if (stop.load(std::memory_order_acquire))
            break;

        protocol::ControlPayload payload;
        const double value = WheelValue(options, sequence, started);
        if (options.wheel == "lr" || options.wheel == "both")
            payload.values[protocol::controlIndex(protocol::ControlValueIndex::ScopeBendLr)] =
                value;
        if (options.wheel == "ud" || options.wheel == "both")
            payload.values[protocol::controlIndex(protocol::ControlValueIndex::ScopeBendUd)] =
                value;

        protocol::Header header;
        header.message_type = protocol::MessageType::RobotControl;
        header.source = protocol::Source::Master;
        header.session_id = session;
        header.sequence = sequence;
        header.sent_at_unix_ns = UnixNowNs();
        protocol::Bytes packet;
        std::string error;
        const bool encoded = protocol::encodeControl(header, payload, packet, &error);
        bool sent = false;
        if (encoded) {
            const int result = sendto(socket_handle,
                                      reinterpret_cast<const char *>(packet.data()),
                                      static_cast<int>(packet.size()),
                                      0,
                                      reinterpret_cast<const sockaddr *>(&target),
                                      sizeof(target));
            sent = result == static_cast<int>(packet.size());
        }
        {
            std::lock_guard<std::mutex> lock(observations_mutex);
            observations.push_back({SteadyNowNs(), header.sent_at_unix_ns, sequence, sent});
        }
        ++sequence;
        next += period;
        if (next < std::chrono::steady_clock::now())
            next = std::chrono::steady_clock::now() + period;
    }
    shutdown(socket_handle, SD_BOTH);
    closesocket(socket_handle);
}

struct Distribution {
    std::size_t count = 0;
    double min = 0;
    double p50 = 0;
    double p95 = 0;
    double max = 0;
    double mean = 0;
};

Distribution Describe(std::vector<double> values)
{
    Distribution result;
    result.count = values.size();
    if (values.empty())
        return result;
    std::sort(values.begin(), values.end());
    result.min = values.front();
    result.max = values.back();
    result.p50 = values[(values.size() - 1) * 50 / 100];
    result.p95 = values[(values.size() - 1) * 95 / 100];
    for (const double value : values)
        result.mean += value;
    result.mean /= static_cast<double>(values.size());
    return result;
}

void PrintDistribution(const char *name, const Distribution &distribution)
{
    std::cout << std::fixed << std::setprecision(3) << name << " count=" << distribution.count
              << " min_ms=" << distribution.min << " p50_ms=" << distribution.p50
              << " p95_ms=" << distribution.p95 << " max_ms=" << distribution.max
              << " mean_ms=" << distribution.mean << '\n';
}

std::vector<double> MasterIntervals(const std::vector<MasterObservation> &observations)
{
    std::vector<double> result;
    std::uint64_t previous = 0;
    for (const auto &observation : observations) {
        if (!observation.sent)
            continue;
        if (previous != 0)
            result.push_back((observation.sent_steady_ns - previous) / 1000000.0);
        previous = observation.sent_steady_ns;
    }
    return result;
}

std::vector<double> StatusIntervals(const std::vector<StatusObservation> &observations)
{
    std::vector<double> result;
    std::uint64_t previous = 0;
    for (const auto &observation : observations) {
        if (previous != 0)
            result.push_back((observation.received_steady_ns - previous) / 1000000.0);
        previous = observation.received_steady_ns;
    }
    return result;
}

std::vector<double> FollowIntervals(const std::vector<timing_probe::AdsRequestRecord> &records)
{
    static constexpr const char *kFollowSymbol =
        "MAIN.Follow_Control_Cmd.Cmd_Follow_Comp_Joy_FromMaster";
    std::vector<double> result;
    std::uint64_t previous = 0;
    for (const auto &record : records) {
        if (!record.is_write || record.symbol != kFollowSymbol || record.ads_error != 0)
            continue;
        if (previous != 0)
            result.push_back((record.received_unix_ns - previous) / 1000000.0);
        previous = record.received_unix_ns;
    }
    return result;
}

std::vector<double> RequestDurations(const std::vector<timing_probe::AdsRequestRecord> &records,
                                     bool writes_only)
{
    std::vector<double> result;
    for (const auto &record : records) {
        if (writes_only && !record.is_write)
            continue;
        if (record.responded_unix_ns >= record.received_unix_ns)
            result.push_back((record.responded_unix_ns - record.received_unix_ns) / 1000000.0);
    }
    return result;
}

std::vector<double> AppliedAges(const std::vector<StatusObservation> &observations)
{
    std::vector<double> result;
    for (const auto &observation : observations) {
        const auto &record = observation.payload.applied_command.last_successful_write;
        if (record.command_sequence == 0 || record.received_unix_ns == 0 ||
            record.applied_unix_ns < record.received_unix_ns)
            continue;
        result.push_back((record.applied_unix_ns - record.received_unix_ns) / 1000000.0);
    }
    return result;
}

void WriteCsv(const Options &options,
              const std::vector<MasterObservation> &master,
              const std::vector<StatusObservation> &status,
              const std::vector<timing_probe::AdsRequestRecord> &ads)
{
    if (options.csv_path.empty())
        return;
    std::ofstream output(options.csv_path, std::ios::trunc);
    if (!output)
        return;
    output << "kind,steady_ns,unix_ns,sequence,symbol,command,group,ads_error,"
              "status_sequence,applied_sequence,applied_age_ms\n";
    for (const auto &sample : master)
        output << "master," << sample.sent_steady_ns << ',' << sample.sent_unix_ns << ','
               << sample.sequence << ",,,,,,,,\n";
    for (const auto &sample : status) {
        const auto &record = sample.payload.applied_command.last_successful_write;
        const double age = record.received_unix_ns != 0 &&
                                   record.applied_unix_ns >= record.received_unix_ns
                               ? (record.applied_unix_ns - record.received_unix_ns) / 1000000.0
                               : -1.0;
        output << "status," << sample.received_steady_ns << ',' << sample.header.sent_at_unix_ns
               << ",,,," << sample.header.sequence << ',' << record.command_sequence << ','
               << age << '\n';
    }
    for (const auto &record : ads)
        output << "ads," << record.received_unix_ns << ',' << record.responded_unix_ns << ",,"
               << record.symbol << ',' << record.command << ',' << record.index_group << ','
               << record.ads_error << ",,,,\n";
}

timing_probe::FakeAdsServer g_fake_ads;

} // namespace

/**
 * @brief 离线 RobotSystem 时序探针入口。
 * @details 先启动 fake ADS，再让真实 RobotSystem 控制/状态线程运行，最后用伪 Master
 *          发送大小拨轮命令并汇总各边界的间隔与延迟分布。
 */
int main(int argc, char **argv)
{
    Options options;
    if (!ParseOptions(argc, argv, options)) {
        if (argc > 1 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h"))
            return 0;
        PrintUsage();
        return 2;
    }

    timing_probe::FakeAdsOptions ads_options;
    ads_options.response_delay_ms = options.ads_delay_ms;
    ads_options.delay_scope = options.ads_delay_scope;
    std::string error;
    if (!g_fake_ads.Start(ads_options, &error)) {
        std::cerr << "fake ADS start failed: " << error << '\n';
        return 3;
    }

    // robot_device.dll resolves its config path during DLL initialization,
    // before main() can change the current directory. The executable is
    // therefore launched from tests/RobotTimingProbe, where config.yaml points
    // to the fixed local fake ADS port.
    StatusCollector status_collector;
    if (!status_collector.Start(kStatusPort, &error)) {
        std::cerr << "status collector start failed: " << error << '\n';
        return 3;
    }

    try {
        if (ercp::LoadSettings() <= 0) {
            std::cerr << "probe settings failed to load\n";
            return 4;
        }

        // GetRobot() initializes the real Beckhoff adapter, but the generated
        // config points it only at the in-process fake ADS TCP server.
        (void)ercp::GetRobot();
        auto &robot_system = ercp::YunSBot::GetInstance();
        if (!robot_system.base.Start(1)) {
            std::cerr << "RobotSystem start request was rejected\n";
            return 5;
        }

        const auto start_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!robot_system.base.IsRobotRunning() &&
               std::chrono::steady_clock::now() < start_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!robot_system.base.IsRobotRunning()) {
            std::cerr << "RobotSystem did not enter running state\n";
            return 5;
        }

        std::atomic<bool> stop_master{false};
        std::vector<MasterObservation> master_observations;
        std::mutex master_mutex;
        std::thread master_thread([&]() {
            RunFakeMaster(options, stop_master, master_observations, master_mutex);
        });

        std::this_thread::sleep_for(std::chrono::milliseconds(options.duration_ms));
        stop_master.store(true, std::memory_order_release);
        if (master_thread.joinable())
            master_thread.join();

        if (robot_system.base.IsRobotRunning())
            robot_system.base.Stop();
        const auto stop_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (robot_system.base.IsRobotRunning() &&
               std::chrono::steady_clock::now() < stop_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        status_collector.Stop();
        std::vector<MasterObservation> master_copy;
        {
            std::lock_guard<std::mutex> lock(master_mutex);
            master_copy = master_observations;
        }
        const auto status_copy = status_collector.Records();
        const auto ads_copy = g_fake_ads.Records();
        const auto master_stats = robot_system.master.Stats();

        std::cout << "mode=offline_fake_ads duration_ms=" << options.duration_ms
                  << " frequency_hz=" << options.frequency_hz
                  << " ads_delay_ms=" << options.ads_delay_ms
                  << " ads_delay_scope=" << DelayScopeName(options.ads_delay_scope) << '\n';
        PrintDistribution("master_tx_interval", Describe(MasterIntervals(master_copy)));
        PrintDistribution("robot_status_interval", Describe(StatusIntervals(status_copy)));
        PrintDistribution("follow_write_interval", Describe(FollowIntervals(ads_copy)));
        PrintDistribution("ads_request_service", Describe(RequestDurations(ads_copy, false)));
        PrintDistribution("ads_write_service", Describe(RequestDurations(ads_copy, true)));
        PrintDistribution("command_applied_age", Describe(AppliedAges(status_copy)));
        std::cout << "counts master_tx="
                  << std::count_if(master_copy.begin(), master_copy.end(), [](const auto &sample) {
                         return sample.sent;
                     })
                  << " status=" << status_copy.size() << " ads_requests=" << ads_copy.size()
                  << " ads_writes="
                  << std::count_if(ads_copy.begin(), ads_copy.end(), [](const auto &record) {
                         return record.is_write;
                     })
                  << "\n";
        std::cout << "master_rx received=" << master_stats.received
                  << " accepted=" << master_stats.accepted
                  << " rejected=" << master_stats.rejected
                  << " duplicate=" << master_stats.duplicate
                  << " out_of_order=" << master_stats.out_of_order
                  << " sequence_gaps=" << master_stats.gaps << "\n";

        const auto follow = Describe(FollowIntervals(ads_copy));
        const auto applied = Describe(AppliedAges(status_copy));
        bool passed = follow.count >= 5 && !status_copy.empty();
        if (options.max_follow_period_ms > 0)
            passed = passed && follow.count != 0 && follow.p95 <= options.max_follow_period_ms;
        if (options.max_applied_age_ms > 0)
            passed = passed && applied.count != 0 && applied.p95 <= options.max_applied_age_ms;
        std::cout << "verdict=" << (passed ? "PASS" : "FAIL") << '\n';

        WriteCsv(options, master_copy, status_copy, ads_copy);
        const int exit_code = passed ? 0 : 6;

        // The production singleton owns Poco UDP servers and a background
        // io_service worker but does not expose a complete shutdown hook.
        // Terminate this short-lived diagnostic process without entering
        // those unrelated static destructors; the OS closes the in-process
        // fake ADS socket and its worker with the process.
        std::cout.flush();
        std::cerr.flush();
        std::_Exit(exit_code);
    } catch (const std::exception &exception) {
        std::cerr << "probe exception: " << exception.what() << '\n';
        return 7;
    }
}
