#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <set>
#include <fmt/format.h>

#include "Device.hpp"
#include "beckhoff_feedback_layout.hpp"
#include "beckhoff_snapshot.hpp"
#include "direct_ads_transport.hpp"
#include "yunsbot_config.h"

#include <windows.h> //windowsAPI的函数声明和宏
#include "TcAdsDef.h" //结构体和常量的声明
#include "TcAdsAPI.h" // ADS函数的声明

namespace device {
namespace beckhoff {
using namespace std;

// =============================================================================

// 倍福电机对象
class Beckhoff_Motor {
public:
    // 获取唯一对象
    static Beckhoff_Motor &GetInstance()
    {
        static Beckhoff_Motor bf;
        return bf;
    }

    using FeedbackData = RobotFeedbackLeaves;

public:
    Beckhoff_Motor();
    ~Beckhoff_Motor();

    // 打开关闭链接
    bool OpenConn(string sIPAddr,
                  int iPort,
                  const string &transport = "twincat",
                  const string &tcpHost = "127.0.0.1",
                  int tcpPort = 48898);
    bool CloseConn();

    bool IsOpen() const { return m_bIsOpen.load(std::memory_order_acquire); }
    BeckhoffSnapshot Snapshot() const;

    // 获取信息
    bool ReadAsexPos(double dAsex_Pos[19]);

    beckhoff_arm_move_state MoveState();

    double Follow_Length();
    double BigWhell();
    double SmallWhell();
    double BigWhellCalc();
    double SmallWhellCalc();
    bool Output_Switch(gpio_output_t out_switch);
    double Force(int iPos);

    // 获取阻力信息
    double DeliverForce();
    // 旋转角度
    double RotateDegree();
    // 抬钳器
    double Lifter();

    // 获取电池信息
    int BatteryInfo();

    // 写入信息
    bool LinearActuator(INT16 data[2]);
    bool MoveArmTo(bool bIsOpen);
    bool FollowOperationData(unsigned long length, void *data);
    std::uint32_t FollowOperationDataResult(unsigned long length, const void *data);
    std::uint32_t GoldDiscreteCommandResult(const device::beckhoff::GoldDiscreteCommand &command);
    bool BaseMoveData(unsigned long length, void *data);
    bool ArmOperation(beckhoff_arm_operation iOpration);

    bool SetEndoscopyType(int iType);

    bool EmergencyStop(bool bIsStop);

    bool ERCPOperateState(bool state); // true = 操作中、False = 未操作

    bool IsERCPOnline();
    bool IsERCPReady();
    double GetERCPDeliverForce();
    double GetERCPGuidwireForce();
    double GetERCPDeliverPos();

private:
    // 读取写入数据
    std::uint32_t ReadData(const char *paraName, unsigned long length, void *data);
    struct AdsReadRequest {
        const char *name;
        unsigned long length;
        void *data;
    };
    std::uint32_t
    ReadDataBatch(const AdsReadRequest *requests, std::size_t count, std::uint32_t *itemErrors);
    std::uint32_t WriteData(const char *paraName, unsigned long length, const void *data);
    std::uint32_t SymbolHandle(const char *paraName, unsigned long &handle);
    void ReleaseSymbolHandles();
    std::uint32_t AdsReadState(std::uint16_t &adsState, std::uint16_t &deviceState);
    std::uint32_t AdsWriteControl(std::uint16_t adsState,
                                  std::uint16_t deviceState,
                                  std::uint32_t length,
                                  const void *data);
    std::uint32_t
    AdsRead(std::uint32_t indexGroup, std::uint32_t indexOffset, std::uint32_t length, void *data);
    std::uint32_t AdsWrite(std::uint32_t indexGroup,
                           std::uint32_t indexOffset,
                           std::uint32_t length,
                           const void *data);
    std::uint32_t AdsReadWrite(std::uint32_t indexGroup,
                               std::uint32_t indexOffset,
                               std::uint32_t readLength,
                               void *readData,
                               std::uint32_t writeLength,
                               const void *writeData,
                               std::uint32_t *bytesRead = nullptr);

    // 创建地址
    bool BuildAddr(string sIP, int iPort, AmsAddr &bfAddr);

    // 更新状态线程
    void StateUpdateThread();
    void PollCommonSnapshot(BeckhoffSnapshot &next, std::string &lastFailureDetails);
    void PollErcpSnapshot(BeckhoffSnapshot &next,
                          std::chrono::steady_clock::time_point cycleStarted,
                          std::chrono::steady_clock::time_point &nextProbe,
                          std::string &lastAvailabilityFailureDetails,
                          std::string &lastStateFailureDetails,
                          std::string &lastFeedbackFailureDetails);
    std::uint32_t PollErcpState(BeckhoffSnapshot &next,
                                std::string &lastFailureDetails);
    std::uint32_t PollErcpFeedback(BeckhoffSnapshot &next,
                                   std::string &lastFailureDetails);
    void PublishSnapshot(BeckhoffSnapshot next);
    boost::shared_ptr<boost::thread> m_StateUpdate_Thread;

    mutable std::mutex m_snapshot_mutex;
    BeckhoffSnapshot m_snapshot;

    mutable std::mutex m_ads_mutex;
    std::map<std::string, unsigned long> m_symbol_handles;
    std::map<std::string, std::uint32_t> m_symbol_errors;
    bool m_sum_read_supported = true;

private:
    // 是否打开连接
    std::atomic<bool> m_bIsOpen{false};
    bool m_port_open = false;
    bool m_direct_mode = false;
    DirectAdsTransport m_direct_ads;
    std::atomic<bool> m_ercp_available{false};
    std::uint32_t m_ercp_failed_polls = 0;
    std::mutex m_command_mutex;
    GoldDiscreteCommand m_last_ercp_command{};
    bool m_has_last_ercp_command = false;

    // 倍福地址
    AmsAddr m_Addr;
};

}
} // namespace device::beckhoff
