#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ercp::timing_probe {

enum class AdsDelayScope {
    None,
    All,
    Reads,
    Writes,
};

struct FakeAdsOptions {
    // Fixed local port keeps the config path deterministic: robot_device.dll
    // resolves config.yaml before main() runs, so the probe is launched from
    // this test directory rather than changing cwd after DLL load.
    std::uint16_t listen_port = 48898;
    int response_delay_ms = 0;
    AdsDelayScope delay_scope = AdsDelayScope::None;
};

struct AdsRequestRecord {
    std::uint64_t received_unix_ns = 0;
    std::uint64_t responded_unix_ns = 0;
    std::uint16_t command = 0;
    std::uint32_t index_group = 0;
    std::uint32_t index_offset = 0;
    std::uint32_t length = 0;
    std::uint32_t ads_error = 0;
    bool is_write = false;
    std::string symbol;
};

/**
 * @brief 仅用于离线时序测试的最小 ADS/AMS TCP 服务。
 * @details 它模拟 RobotSystem 当前 direct ADS 传输所需的读状态、按名称取句柄、
 *          Sum Read、句柄读写和释放句柄，不连接 TwinCAT，也不执行任何真实电机动作。
 */
class FakeAdsServer {
public:
    FakeAdsServer();
    ~FakeAdsServer();

    FakeAdsServer(const FakeAdsServer &) = delete;
    FakeAdsServer &operator=(const FakeAdsServer &) = delete;

    bool Start(const FakeAdsOptions &options, std::string *error = nullptr);
    void Stop();

    std::uint16_t port() const;
    std::vector<AdsRequestRecord> Records() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ercp::timing_probe
