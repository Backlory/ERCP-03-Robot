#pragma once
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <iostream>
#include "robot_config.h"
#include "utils.h"
#include "beckhoff_driver.hpp"
#include "beckhoff_snapshot_policy.hpp"

namespace device {
namespace beckhoff {

namespace {

std::uint64_t UnixNowNs()
{
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count());
}

void KeepFirstError(std::uint32_t &current, std::uint32_t candidate)
{
    if (current == ADSERR_NOERR && candidate != ADSERR_NOERR)
        current = candidate;
}

template <std::size_t Count>
std::array<std::string, Count> MakeIndexedSymbolNames(const char *prefix)
{
    std::array<std::string, Count> names{};
    for (std::size_t i = 0; i < Count; ++i)
        names[i] = std::string(prefix) + std::to_string(i + 1) + "]";
    return names;
}

template <typename ErrorContainer>
bool AnySuccessful(const ErrorContainer &errors)
{
    return std::any_of(errors.begin(), errors.end(), [](std::uint32_t error) {
        return error == ADSERR_NOERR;
    });
}

template <typename RequestContainer, typename ErrorContainer>
std::string FormatAdsReadFailures(const RequestContainer &requests,
                                  std::size_t count,
                                  const ErrorContainer &errors)
{
    std::ostringstream failures;
    bool first = true;
    for (std::size_t i = 0; i < count; ++i) {
        const auto error = errors[i];
        if (error == ADSERR_NOERR)
            continue;
        if (!first)
            failures << ", ";
        failures << requests[i].name << "=0x" << std::uppercase << std::hex << error
                 << std::nouppercase << std::dec;
        if (error == ADSERR_DEVICE_SYMBOLNOTFOUND)
            failures << "(ADS_SYMBOL_NOT_FOUND)";
        first = false;
    }
    return failures.str();
}

constexpr std::size_t kMainMotorErrorCount = 19;
constexpr std::size_t kCommonBaseRequestCount = 3 + kMainMotorErrorCount;
constexpr std::size_t kCommonRequestCount =
    kCommonBaseRequestCount + kRobotFeedbackLeafCount;

const auto kMainMotorErrorNames =
    MakeIndexedSymbolNames<kMainMotorErrorCount>("MAIN.MotorErrorState[");
const auto kAxesPositionNames = MakeIndexedSymbolNames<kRobotPublishedAxisCount>(
    "MAIN.Info_Feedback_ToMaster.Axes_Pos[");
const auto kForceSensorNames = MakeIndexedSymbolNames<kRobotForceSensorCount>(
    "MAIN.Info_Feedback_ToMaster.Force_Sensor[");

constexpr std::size_t kErcpDriveErrorCount = 13;
constexpr std::size_t kErcpMotorErrorCount = 11;
constexpr std::size_t kErcpDriveErrorsRequestStart = 3;
constexpr std::size_t kErcpMotorErrorFlagRequestIndex =
    kErcpDriveErrorsRequestStart + kErcpDriveErrorCount;
constexpr std::size_t kErcpMotorErrorsRequestStart = kErcpMotorErrorFlagRequestIndex + 1;
constexpr std::size_t kErcpTypeRequestIndex =
    kErcpMotorErrorsRequestStart + kErcpMotorErrorCount;
constexpr std::size_t kErcpMoveStatusRequestIndex = kErcpTypeRequestIndex + 1;
constexpr std::size_t kErcpLoadDirectionRequestIndex = kErcpMoveStatusRequestIndex + 1;
constexpr std::size_t kErcpStateRequestCount = kErcpLoadDirectionRequestIndex + 1;

// Interface presence is determined by the ERCP readiness symbol, not by an
// optional force-feedback leaf. A successful read means the interface exists;
// the BOOL value itself is published separately as the ready flag.
constexpr const char *kErcpAvailabilitySymbol =
    "POU_Ercp_CycleExecute.Ercp_Ready_State";

const auto kErcpDriveErrorNames =
    MakeIndexedSymbolNames<kErcpDriveErrorCount>("MAIN_ERCP.DriveErrorState_ERCP[");
const auto kErcpMotorErrorNames =
    MakeIndexedSymbolNames<kErcpMotorErrorCount>("MAIN_ERCP.MotorErrorState_ERCP[");

} // namespace

// =============================================================================
// 倍福电机对象

Beckhoff_Motor::Beckhoff_Motor() = default;
Beckhoff_Motor::~Beckhoff_Motor()
{
    CloseConn();
}

// 打开链接
/**
 * @brief 功能：建立 Beckhoff ADS 连接，确认 PLC 可通信并启动状态轮询线程。
 * @details 机制：先初始化目标地址和 direct/原生传输，再读取 PLC 状态但不改变其运行状态，最后探测 ERCP 可选符号并启动轮询。
 */
bool Beckhoff_Motor::OpenConn(string sIPAddr,
                              int iPort,
                              const string &transport,
                              const string &tcpHost,
                              int tcpPort)
{
    if (IsOpen())
        return true;
    CloseConn();
    m_ercp_available.store(false, std::memory_order_release);
    m_ercp_failed_polls = 0;

    {
        std::lock_guard<std::mutex> lock(m_snapshot_mutex);
        m_snapshot.connection_state = SnapshotConnectionState::Connecting;
        m_snapshot.published_unix_ns = UnixNowNs();
    }

    long nPort, nErr;
    USHORT nAdsState; //包含PLC的状态信�?
    // 倍福地址
    // AmsAddr bfAddr;// = BuildAddr(sIPAddr, iPort);

    BuildAddr(sIPAddr, iPort, m_Addr);
    m_direct_mode = transport == "direct";
    {
        std::lock_guard<std::mutex> lock(m_ads_mutex);
        if (m_direct_mode) {
            m_port_open =
                tcpPort > 0 && tcpPort <= 65535 &&
                m_direct_ads.Connect(tcpHost, static_cast<std::uint16_t>(tcpPort), m_Addr);
            nPort = m_port_open ? 1 : 0;
        } else {
            nPort = AdsPortOpen();
            m_port_open = nPort > 0;
        }
    }
    if (!m_port_open) {
        std::lock_guard<std::mutex> lock(m_snapshot_mutex);
        m_snapshot.connection_state = SnapshotConnectionState::Disconnected;
        m_snapshot.overall_ads_error = ADSERR_CLIENT_PORTNOTOPEN;
        m_snapshot.published_unix_ns = UnixNowNs();
        return false;
    }
    //		nErr = AdsGetLocalAddress(&bfAddr);
    // if (nErr) {
    //	//printf(m_lastError, "Error: AdsGetLocalAddress: %d\n", nErr);
    //	return false;
    //}
    // bfAddr.port = 851;

    USHORT nDeviceState;
    {
        std::lock_guard<std::mutex> lock(m_ads_mutex);
        nErr = AdsReadState(nAdsState, nDeviceState);
    }
    if (nErr) {
        //(m_lastError, "Error: AdsSyncReadStateReq:  %d\n", nErr);
        {
            std::lock_guard<std::mutex> lock(m_snapshot_mutex);
            m_snapshot.overall_ads_error = static_cast<std::uint32_t>(nErr);
        }
        CloseConn();
        return false;
    }
    // Connecting to ADS must not start the PLC. Keep the state read above as a
    // communication check and leave STOP/RUN control to the operator.
    m_bIsOpen.store(true, std::memory_order_release);
    m_sum_read_supported = true;
    // MAIN_ERCP is optional. Probe the readiness symbol; a successful read
    // means the interface exists even when the cart is currently not ready.
    bool ercpReadyProbe = false;
    m_ercp_available.store(ReadData(kErcpAvailabilitySymbol,
                                    kAdsBoolBytes,
                                    &ercpReadyProbe) == ADSERR_NOERR,
                           std::memory_order_release);
    m_StateUpdate_Thread =
        boost::make_shared<boost::thread>(&Beckhoff_Motor::StateUpdateThread, this);
    return true;
}

// 关闭链接
/**
 * @brief 功能：停止 ADS 状态线程、释放符号句柄和关闭传输连接。
 * @details 机制：先撤销打开标志并 join 轮询线程，再在 ADS 锁内关闭端口，最后把快照标记为断开和过期。
 */
bool Beckhoff_Motor::CloseConn()
{
    m_bIsOpen.store(false, std::memory_order_release);

    if (m_StateUpdate_Thread) {
        m_StateUpdate_Thread->interrupt();
        m_StateUpdate_Thread->join();
        m_StateUpdate_Thread.reset();
    }

    ReleaseSymbolHandles();

    long nErr = ADSERR_NOERR;
    {
        std::lock_guard<std::mutex> lock(m_ads_mutex);
        if (m_port_open) {
            if (m_direct_mode) {
                m_direct_ads.Close();
            } else {
                nErr = AdsPortClose();
            }
            m_port_open = false;
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_snapshot_mutex);
        m_snapshot.connection_state = SnapshotConnectionState::Disconnected;
        m_snapshot.stale_groups |= m_snapshot.valid_groups;
        m_snapshot.published_unix_ns = UnixNowNs();
    }

    return nErr == ADSERR_NOERR;
}

// 设置水气
bool Beckhoff_Motor::LinearActuator(INT16 data[2])
{
    return WriteData("MAIN.ILA_Node3", 4, data) == ADSERR_NOERR;
}

// 写入
bool Beckhoff_Motor::MoveArmTo(bool bIsOpen)
{
    beckhoff_arm_operation iMoveState = BAO_FOLD;
    if (bIsOpen)
        iMoveState = BAO_OPEN;

    return ArmOperation(iMoveState);
}

// 跟随数据发�?
bool Beckhoff_Motor::FollowOperationData(unsigned long length, void *data)
{
    return FollowOperationDataResult(length, data) == ADSERR_NOERR;
}

/**
 * @brief 功能：把 typed follow 命令拆成 PLC 需要的操作、回零和 IO 字段并写入 ADS。
 * @details 机制：先校验结构体长度，再按固定 symbol 分组写入；通过 KeepFirstError 汇总首个失败码并更新命令写入诊断。
 */
std::uint32_t Beckhoff_Motor::FollowOperationDataResult(unsigned long length, const void *data)
{
    if (length != sizeof(beckhoff_follow_cmd) || data == nullptr)
        return ADSERR_DEVICE_INVALIDSIZE;
    const auto &command = *static_cast<const beckhoff_follow_cmd *>(data);
    const double operatorValues[9] = {command.vel_move,
                                      command.vel_rotate,
                                      command.vel_bend_lr,
                                      command.vel_bend_ud,
                                      command.vel_pincer,
                                      command.vel_cutter_feed,
                                      command.vel_cutter_rot,
                                      command.vel_cutter_bend,
                                      command.vel_wire_feed};
    const bool homeValues[3] = {command.home_rotate, command.home_bend_lr, command.home_bend_ud};
    const bool ioValues[3] = {command.switch_water, command.switch_gas, command.switch_suct};
    std::uint32_t result = ADSERR_NOERR;
    KeepFirstError(result,
                   WriteData("MAIN.Follow_Control_Cmd.Cmd_Follow_Comp_Joy_FromMaster",
                             sizeof(command.follow_comp_botton),
                             &command.follow_comp_botton));
    KeepFirstError(result,
                   WriteData("MAIN.Follow_Control_Cmd.Cmd_Operator_Joy_FromMaster",
                             sizeof(operatorValues),
                             operatorValues));
    KeepFirstError(result,
                   WriteData("MAIN.Follow_Control_Cmd.Cmd_Home_Joy_FromMaster",
                             sizeof(homeValues),
                             homeValues));
    KeepFirstError(
        result,
        WriteData("MAIN.Follow_Control_Cmd.Cmd_IO_Joy_FromMaster", sizeof(ioValues), ioValues));
    std::lock_guard<std::mutex> lock(m_snapshot_mutex);
    m_snapshot.command_write_ads_error = result;
    return result;
}

/**
 * @brief 功能：按变化检测将离散控制命令和 ERCP 扩展字段写入 PLC。
 * @details 机制：先写机器人动作，再依据 ERCP 可用性和上一条命令只写变化字段；失败时清除缓存，迫使下一次完整重写。
 */
std::uint32_t
Beckhoff_Motor::GoldDiscreteCommandResult(const device::beckhoff::GoldDiscreteCommand &command)
{
    std::uint32_t result = ADSERR_NOERR;
    if (command.robot_action >= 0) {
        KeepFirstError(result,
                       WriteData("MAIN.Status_Command_FromMaster",
                                 sizeof(command.robot_action),
                                 &command.robot_action));
    }
    if (!m_ercp_available.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        m_has_last_ercp_command = false;
        return result;
    }

    auto applied = command;
    const auto snapshot = Snapshot();
    if (snapshot.inject_state_01 == 11)
        applied.inject_enable[0] = false;
    if (snapshot.inject_state_02 == 11)
        applied.inject_enable[1] = false;

    std::lock_guard<std::mutex> lock(m_command_mutex);
    const bool first = !m_has_last_ercp_command;
    const auto &last = m_last_ercp_command;
    if (first || applied.operate != last.operate)
        KeepFirstError(result,
                       WriteData("MAIN_ERCP.bERCP_Operate_State_FromMaster",
                                 sizeof(applied.operate),
                                 &applied.operate));
    if (first || applied.cooperate != last.cooperate)
        KeepFirstError(result,
                       WriteData("MAIN_ERCP.bErcp_Cooperate_Enable",
                                 sizeof(applied.cooperate),
                                 &applied.cooperate));
    if (first || std::memcmp(applied.handle_6d, last.handle_6d, sizeof(applied.handle_6d)) != 0)
        KeepFirstError(result,
                       WriteData("MAIN_ERCP.ERCP_Control_Cmd.Cmd_6Dhandle_Joy_FromMaster",
                                 sizeof(applied.handle_6d),
                                 applied.handle_6d));
    if (first || std::memcmp(applied.buttons, last.buttons, sizeof(applied.buttons)) != 0)
        KeepFirstError(result,
                       WriteData("MAIN_ERCP.ERCP_Control_Cmd.Cmd_Button_Joy_FromMaster",
                                 sizeof(applied.buttons),
                                 applied.buttons));
    static constexpr const char *velocitySymbols[2] = {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Vel_01",
                                                       "MAIN_ERCP.ERCP_Inject_Cmd.Inject_Vel_02"};
    static constexpr const char *positionSymbols[2] = {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Pos_01",
                                                       "MAIN_ERCP.ERCP_Inject_Cmd.Inject_Pos_02"};
    static constexpr const char *enableSymbols[2] = {"MAIN_ERCP.ERCP_Inject_Cmd.Inject_Enable_01",
                                                     "MAIN_ERCP.ERCP_Inject_Cmd.Inject_Enable_02"};
    for (std::size_t i = 0; i < 2; ++i) {
        if (first || applied.inject_velocity[i] != last.inject_velocity[i])
            KeepFirstError(result,
                           WriteData(velocitySymbols[i],
                                     sizeof(applied.inject_velocity[i]),
                                     &applied.inject_velocity[i]));
        if (first || applied.inject_position[i] != last.inject_position[i])
            KeepFirstError(result,
                           WriteData(positionSymbols[i],
                                     sizeof(applied.inject_position[i]),
                                     &applied.inject_position[i]));
        if (first || applied.inject_enable[i] != last.inject_enable[i])
            KeepFirstError(result,
                           WriteData(enableSymbols[i],
                                     sizeof(applied.inject_enable[i]),
                                     &applied.inject_enable[i]));
    }
    if (result == ADSERR_NOERR) {
        m_last_ercp_command = applied;
        m_has_last_ercp_command = true;
    } else {
        m_has_last_ercp_command = false;
    }
    return result;
}

// ��������
bool Beckhoff_Motor::BaseMoveData(unsigned long length, void *data)
{
    return WriteData("MAIN.Base_Control_Cmd", length, data) == ADSERR_NOERR;
}

bool Beckhoff_Motor::ArmOperation(beckhoff_arm_operation iOpration)
{
    return WriteData("MAIN.Status_Command_FromMaster", 4, &iOpration) == ADSERR_NOERR;
}

bool Beckhoff_Motor::SetEndoscopyType(int iType)
{
    return WriteData("MAIN.type_of_scope", 4, &iType) == ADSERR_NOERR;
}

/**
 * @brief 执行或解除机器人的急停状态。
 * @details 急停只写入 PLC 专用急停变量，不触碰运动状态或 Status_Command_FromMaster。
 *          当前 TwinCAT 工程约定 MAIN.Emergency_Stop_FromMaster 为高有效：1 表示急停，0 表示恢复。
 */
bool Beckhoff_Motor::EmergencyStop(bool bIsStop)
{
    // Domain semantics and the PLC variable use the same polarity:
    // true/1 asserts the emergency stop, false/0 releases it.
    int emergencyStopSignal = bIsStop ? 1 : 0;
    return WriteData("MAIN.Emergency_Stop_FromMaster", 4, &emergencyStopSignal)
        == ADSERR_NOERR;
}

// 读取
BeckhoffSnapshot Beckhoff_Motor::Snapshot() const
{
    std::lock_guard<std::mutex> lock(m_snapshot_mutex);
    return m_snapshot;
}

beckhoff_arm_move_state Beckhoff_Motor::MoveState()
{
    return static_cast<beckhoff_arm_move_state>(Snapshot().move_state);
}

double Beckhoff_Motor::Follow_Length()
{
    return Snapshot().common_values[0];
}

double Beckhoff_Motor::SmallWhell()
{
    return Snapshot().common_values[17 + 16];
}
double Beckhoff_Motor::BigWhell()
{
    return Snapshot().common_values[17 + 17];
}

double Beckhoff_Motor::SmallWhellCalc()
{
    return Snapshot().common_values[2];
}
double Beckhoff_Motor::BigWhellCalc()
{
    return Snapshot().common_values[1];
}

/**
 * @brief 查询指定 GPIO 输出在最新 Beckhoff 快照中的开关状态。
 * @details 将 gas、water、suct 三种逻辑输出映射到快照中的固定 bit；未知输出统一返回关闭。
 */
bool Beckhoff_Motor::Output_Switch(gpio_output_t out_switch)
{
    // 从一致快照读取输出位，并把 gas/water/suction 映射为对应的 PLC bit。
    const auto switches = Snapshot().output_switches;
    if (gpio_output_t::gas == out_switch)
        return (switches & (1u << 1)) != 0;
    else if (gpio_output_t::water == out_switch)
        return (switches & (1u << 0)) != 0;
    else if (gpio_output_t::suct == out_switch)
        return (switches & (1u << 2)) != 0;

    return false;
}

double Beckhoff_Motor::Force(int iPos)
{
    if (iPos < 0 || iPos > 9)
        return 0;
    return Snapshot().common_values[3 + iPos];
}

int Beckhoff_Motor::BatteryInfo()
{
    return Snapshot().power_level;
}

// 获取阻力信息
double Beckhoff_Motor::DeliverForce()
{
    return Snapshot().common_values[14];
}
// 旋转角度
double Beckhoff_Motor::RotateDegree()
{
    return Snapshot().common_values[15];
}
// 抬钳�?
double Beckhoff_Motor::Lifter()
{
    return Snapshot().common_values[13];
}

bool Beckhoff_Motor::ReadAsexPos(double dAsex_Pos[19])
{
    const auto snapshot = Snapshot();
    for (int i = 0; i < 19; i++) {
        dAsex_Pos[i] = snapshot.common_values[17 + i];
    }
    return true;
}
// 读取编码
bool Beckhoff_Motor::ERCPOperateState(bool state) // true = 操作中、False = 未操�?
{
    return WriteData("MAIN_ERCP.bERCP_Operate_State_FromMaster", sizeof(state), &state) ==
           ADSERR_NOERR;
}

bool Beckhoff_Motor::IsERCPOnline()
{
    const auto snapshot = Snapshot();
    return (snapshot.valid_groups & SnapshotErcpState) != 0 &&
           (snapshot.stale_groups & SnapshotErcpState) == 0 &&
           (snapshot.ercp_flags & (1u << 0)) != 0;
}

bool Beckhoff_Motor::IsERCPReady()
{
    const auto snapshot = Snapshot();
    return (snapshot.valid_groups & SnapshotErcpState) != 0 &&
           (snapshot.stale_groups & SnapshotErcpState) == 0 &&
           (snapshot.ercp_flags & (1u << 1)) != 0;
}

double Beckhoff_Motor::GetERCPDeliverForce()
{
    return Snapshot().ercp_deliver_force;
}

double Beckhoff_Motor::GetERCPGuidwireForce()
{
    return Snapshot().guide_wire_force;
}

double Beckhoff_Motor::GetERCPDeliverPos()
{
    return Snapshot().ercp_deliver_position;
}

// ================================================================================

std::uint32_t Beckhoff_Motor::AdsReadState(std::uint16_t &adsState, std::uint16_t &deviceState)
{
    if (m_direct_mode)
        return m_direct_ads.ReadState(adsState, deviceState);
    return static_cast<std::uint32_t>(AdsSyncReadStateReq(&m_Addr, &adsState, &deviceState));
}

std::uint32_t Beckhoff_Motor::AdsWriteControl(std::uint16_t adsState,
                                              std::uint16_t deviceState,
                                              std::uint32_t length,
                                              const void *data)
{
    if (m_direct_mode)
        return m_direct_ads.WriteControl(adsState, deviceState, length, data);
    return static_cast<std::uint32_t>(
        AdsSyncWriteControlReq(&m_Addr, adsState, deviceState, length, const_cast<void *>(data)));
}

std::uint32_t Beckhoff_Motor::AdsRead(std::uint32_t indexGroup,
                                      std::uint32_t indexOffset,
                                      std::uint32_t length,
                                      void *data)
{
    if (m_direct_mode)
        return m_direct_ads.Read(indexGroup, indexOffset, length, data);
    return static_cast<std::uint32_t>(
        AdsSyncReadReq(&m_Addr, indexGroup, indexOffset, length, data));
}

std::uint32_t Beckhoff_Motor::AdsWrite(std::uint32_t indexGroup,
                                       std::uint32_t indexOffset,
                                       std::uint32_t length,
                                       const void *data)
{
    if (m_direct_mode)
        return m_direct_ads.Write(indexGroup, indexOffset, length, data);
    return static_cast<std::uint32_t>(
        AdsSyncWriteReq(&m_Addr, indexGroup, indexOffset, length, const_cast<void *>(data)));
}

/**
 * @brief 功能：根据传输模式执行 ADS ReadWrite，并返回实际读取字节数。
 * @details 机制：direct 模式委托 TCP 适配器；TwinCAT 模式调用原生 ADS API，再把两种实现的返回值统一起来。
 */
std::uint32_t Beckhoff_Motor::AdsReadWrite(std::uint32_t indexGroup,
                                           std::uint32_t indexOffset,
                                           std::uint32_t readLength,
                                           void *readData,
                                           std::uint32_t writeLength,
                                           const void *writeData,
                                           std::uint32_t *bytesRead)
{
    if (m_direct_mode) {
        return m_direct_ads.ReadWrite(indexGroup,
                                      indexOffset,
                                      readLength,
                                      readData,
                                      writeLength,
                                      writeData,
                                      bytesRead);
    }
    unsigned long nativeBytesRead = 0;
    const auto result =
        static_cast<std::uint32_t>(AdsSyncReadWriteReqEx(&m_Addr,
                                                         indexGroup,
                                                         indexOffset,
                                                         readLength,
                                                         readData,
                                                         writeLength,
                                                         const_cast<void *>(writeData),
                                                         &nativeBytesRead));
    if (bytesRead != nullptr)
        *bytesRead = static_cast<std::uint32_t>(nativeBytesRead);
    return result;
}

/**
 * @brief 功能：获取并缓存 PLC symbol 的句柄或稳定的“未找到”错误。
 * @details 机制：先查成功/失败缓存，再按名称解析；旧 PLC 缺少可选叶字段时避免每轮重复查询。
 */
std::uint32_t Beckhoff_Motor::SymbolHandle(const char *paraName, unsigned long &handle)
{
    const auto existing = m_symbol_handles.find(paraName);
    if (existing != m_symbol_handles.end()) {
        handle = existing->second;
        return ADSERR_NOERR;
    }
    const auto failed = m_symbol_errors.find(paraName);
    if (failed != m_symbol_errors.end())
        return failed->second;

    if (!m_port_open)
        return ADSERR_CLIENT_PORTNOTOPEN;

    const auto nameLength = static_cast<unsigned long>(std::strlen(paraName) + 1);
    const auto result =
        AdsReadWrite(ADSIGRP_SYM_HNDBYNAME, 0, sizeof(handle), &handle, nameLength, paraName);
    if (result == ADSERR_NOERR) {
        m_symbol_handles.emplace(paraName, handle);
        m_symbol_errors.erase(paraName);
    } else if (result == ADSERR_DEVICE_SYMBOLNOTFOUND) {
        // Missing leaf symbols are expected on older PLC versions. Cache only
        // the stable not-found result for this connection so a shorter array
        // does not trigger another handle request every 20 ms.
        m_symbol_errors.emplace(paraName, static_cast<std::uint32_t>(result));
    }
    return result;
}

/**
 * @brief 功能：批量读取 PLC symbol，并为每个请求记录独立错误码。
 * @details 机制：先解析/缓存 symbol handle，优先执行 ADS Sum Read；PLC 不支持时永久降级为顺序读取，单项失败不影响其他成功字段。
 */
std::uint32_t Beckhoff_Motor::ReadDataBatch(const AdsReadRequest *requests,
                                            std::size_t count,
                                            std::uint32_t *itemErrors)
{
    if (requests == nullptr || itemErrors == nullptr || count == 0)
        return ADSERR_DEVICE_INVALIDPARM;

    std::fill_n(itemErrors, count, static_cast<std::uint32_t>(ADSERR_NOERR));

    const auto readSequentially = [&]() {
        std::uint32_t firstError = ADSERR_NOERR;
        for (std::size_t i = 0; i < count; ++i) {
            itemErrors[i] = ReadData(requests[i].name, requests[i].length, requests[i].data);
            KeepFirstError(firstError, itemErrors[i]);
        }
        return firstError;
    };

    if (!m_sum_read_supported)
        return readSequentially();
    if (!IsOpen()) {
        std::fill_n(itemErrors, count, static_cast<std::uint32_t>(ADSERR_CLIENT_PORTNOTOPEN));
        return ADSERR_CLIENT_PORTNOTOPEN;
    }

    std::vector<unsigned long> handles(count, 0);
    std::vector<std::size_t> validIndices;
    validIndices.reserve(count);
    std::size_t payloadSize = 0;
    bool fallBackToSequential = false;
    std::uint32_t firstError = ADSERR_NOERR;
    {
        std::lock_guard<std::mutex> lock(m_ads_mutex);
        if (!m_port_open) {
            std::fill_n(itemErrors, count, static_cast<std::uint32_t>(ADSERR_CLIENT_PORTNOTOPEN));
            return ADSERR_CLIENT_PORTNOTOPEN;
        }

        for (std::size_t i = 0; i < count; ++i) {
            itemErrors[i] = SymbolHandle(requests[i].name, handles[i]);
            KeepFirstError(firstError, itemErrors[i]);
            if (itemErrors[i] == ADSERR_NOERR) {
                validIndices.push_back(i);
                payloadSize += requests[i].length;
            }
        }
        if (validIndices.empty())
            return firstError;

        std::vector<std::uint32_t> sumRequest(validIndices.size() * 3, 0);
        for (std::size_t i = 0; i < validIndices.size(); ++i) {
            const auto requestIndex = validIndices[i];
            sumRequest[i * 3] = ADSIGRP_SYM_VALBYHND;
            sumRequest[i * 3 + 1] = static_cast<std::uint32_t>(handles[requestIndex]);
            sumRequest[i * 3 + 2] = static_cast<std::uint32_t>(requests[requestIndex].length);
        }

        const auto resultBytes = validIndices.size() * sizeof(std::uint32_t) + payloadSize;
        std::vector<std::uint8_t> sumResult(resultBytes, 0);
        const auto result =
            AdsReadWrite(ADSIGRP_SUMUP_READ,
                         static_cast<unsigned long>(validIndices.size()),
                         static_cast<unsigned long>(sumResult.size()),
                         sumResult.data(),
                         static_cast<unsigned long>(sumRequest.size() * sizeof(std::uint32_t)),
                         sumRequest.data());
        if (result == ADSERR_DEVICE_SRVNOTSUPP || result == ADSERR_DEVICE_INVALIDGRP) {
            m_sum_read_supported = false;
            fallBackToSequential = true;
        } else if (result != ADSERR_NOERR) {
            for (const auto requestIndex : validIndices)
                itemErrors[requestIndex] = result;
            return result;
        } else {
            std::size_t dataOffset = validIndices.size() * sizeof(std::uint32_t);
            for (std::size_t i = 0; i < validIndices.size(); ++i) {
                const auto requestIndex = validIndices[i];
                std::memcpy(&itemErrors[requestIndex],
                            sumResult.data() + i * sizeof(std::uint32_t),
                            sizeof(itemErrors[requestIndex]));
                if (itemErrors[requestIndex] == ADSERR_NOERR) {
                    std::memcpy(requests[requestIndex].data,
                                sumResult.data() + dataOffset,
                                requests[requestIndex].length);
                } else {
                    AdsWrite(ADSIGRP_SYM_RELEASEHND,
                             0,
                             sizeof(handles[requestIndex]),
                             &handles[requestIndex]);
                    m_symbol_handles.erase(requests[requestIndex].name);
                }
                KeepFirstError(firstError, itemErrors[requestIndex]);
                dataOffset += requests[requestIndex].length;
            }
            return firstError;
        }
    }

    if (fallBackToSequential) {
        ROBOT_INFO(true, "Beckhoff ADS Sum Read is unavailable; falling back to sequential reads.")
        return readSequentially();
    }
    return firstError;
}

// 读取数据
/**
 * @brief 功能：通过缓存的 symbol handle 读取单个 PLC 字段。
 * @details 机制：串行保护 ADS 传输和句柄表；读失败时释放失效句柄，下一次读取会重新解析 symbol。
 */
std::uint32_t Beckhoff_Motor::ReadData(const char *paraName, unsigned long length, void *data)
{
    if (!IsOpen())
        return ADSERR_CLIENT_PORTNOTOPEN;

    std::lock_guard<std::mutex> lock(m_ads_mutex);
    unsigned long handle = 0;
    const auto handleResult = SymbolHandle(paraName, handle);
    if (handleResult != ADSERR_NOERR)
        return handleResult;

    const auto result = AdsRead(ADSIGRP_SYM_VALBYHND, handle, length, data);
    if (result != ADSERR_NOERR) {
        AdsWrite(ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle), &handle);
        m_symbol_handles.erase(paraName);
    }
    return result;
}

// 写入数据
/**
 * @brief 功能：通过缓存的 symbol handle 写入单个 PLC 字段。
 * @details 机制：复用句柄解析和 ADS 锁；写失败时删除句柄缓存，避免后续继续使用失效引用。
 */
std::uint32_t
Beckhoff_Motor::WriteData(const char *paraName, unsigned long length, const void *data)
{
    if (!IsOpen())
        return ADSERR_CLIENT_PORTNOTOPEN;

    std::lock_guard<std::mutex> lock(m_ads_mutex);
    unsigned long handle = 0;
    const auto handleResult = SymbolHandle(paraName, handle);
    if (handleResult != ADSERR_NOERR)
        return handleResult;

    const auto result = AdsWrite(ADSIGRP_SYM_VALBYHND, handle, length, data);
    if (result != ADSERR_NOERR) {
        AdsWrite(ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle), &handle);
        m_symbol_handles.erase(paraName);
    }
    return result;
}

/**
 * @brief 功能：释放当前连接创建的所有 PLC symbol 句柄并清空本地缓存。
 * @details 机制：连接仍开放时逐项发送释放请求，随后无条件清除成功/失败缓存，供下一连接重新解析。
 */
void Beckhoff_Motor::ReleaseSymbolHandles()
{
    std::lock_guard<std::mutex> lock(m_ads_mutex);
    if (m_port_open) {
        for (const auto &entry : m_symbol_handles) {
            auto handle = entry.second;
            AdsWrite(ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle), &handle);
        }
    }
    m_symbol_handles.clear();
    m_symbol_errors.clear();
}

// 建立地址
/**
 * @brief 功能：把文本地址和端口转换为 Beckhoff AMS 目标地址。
 * @details 机制：按点分隔字段填充 NetId；地址不完整时使用本机地址，端口为零时采用 ADS 默认端口 851。
 */
bool Beckhoff_Motor::BuildAddr(string sIP, int iPort, AmsAddr &bfAddr)
{
    auto vIp = ilsr::split(sIP, ".");
    if (vIp.size() >= 4) {
        bfAddr.netId.b[0] = atoi(vIp[0].c_str());
        bfAddr.netId.b[1] = atoi(vIp[1].c_str());
        bfAddr.netId.b[2] = atoi(vIp[2].c_str());
        bfAddr.netId.b[3] = atoi(vIp[3].c_str());
        if (vIp.size() >= 6) {
            bfAddr.netId.b[4] = atoi(vIp[4].c_str());
            bfAddr.netId.b[5] = atoi(vIp[5].c_str());
        } else {
            bfAddr.netId.b[4] = 1;
            bfAddr.netId.b[5] = 1;
        }
    } else {
        // 没有地址，获取本机地址
        AdsGetLocalAddress(&bfAddr);
    }
    // 配置端口
    bfAddr.port = 0 == iPort ? 851 : iPort;

    return true;
}

/**
 * @brief 功能：按公共 PLC 字段读取计划构造 Beckhoff 通用状态快照。
 * @details 机制：先集中登记固定 symbol 与目标内存，再批量读取并聚合每项错误，最后只提交成功字段并更新 valid/stale 诊断。
 */
void Beckhoff_Motor::PollCommonSnapshot(BeckhoffSnapshot &next,
                                        std::string &lastFailureDetails)
{
    std::uint32_t moveState{};
    std::int16_t prepareState{};
    std::int32_t scopeType{};
    FeedbackData feedback{};
    RobotFeedbackLeafErrors feedbackErrors{};
    std::array<bool, kMainMotorErrorCount> mainMotorErrors{};
    std::array<AdsReadRequest, kCommonRequestCount> requests{};
    std::array<std::uint32_t, kCommonRequestCount> itemErrors{};
    std::size_t requestCount = 0;
    const auto add = [&](const char *name, unsigned long length, void *data) {
        requests[requestCount++] = {name, length, data};
    };

    add("MAIN.Status_Feedback_ToMaster", kAdsInt32Bytes, &moveState);
    add("MAIN.iPrepare_State", kAdsInt16Bytes, &prepareState);
    add("MAIN.type_of_scope", kAdsInt32Bytes, &scopeType);
    for (std::size_t i = 0; i < kMainMotorErrorCount; ++i)
        add(kMainMotorErrorNames[i].c_str(), kAdsBoolBytes, &mainMotorErrors[i]);

    const auto feedbackStart = requestCount;
    add("MAIN.Info_Feedback_ToMaster.Follow_Length", kAdsLrealBytes, &feedback.follow_length);
    add("MAIN.Info_Feedback_ToMaster.Switch_Water", kAdsBoolBytes, &feedback.switch_water);
    add("MAIN.Info_Feedback_ToMaster.Switch_Gas", kAdsBoolBytes, &feedback.switch_gas);
    add("MAIN.Info_Feedback_ToMaster.Switch_Suck", kAdsBoolBytes, &feedback.switch_suck);
    add("MAIN.Info_Feedback_ToMaster.Big_Wheel", kAdsLrealBytes, &feedback.big_wheel);
    add("MAIN.Info_Feedback_ToMaster.Small_Wheel", kAdsLrealBytes, &feedback.small_wheel);
    for (std::size_t i = 0; i < kRobotForceSensorCount; ++i)
        add(kForceSensorNames[i].c_str(), kAdsLrealBytes, &feedback.force_sensor[i]);
    add("MAIN.Info_Feedback_ToMaster.Power_level", kAdsInt16Bytes, &feedback.power_level);
    add("MAIN.Info_Feedback_ToMaster.lifter", kAdsLrealBytes, &feedback.lifter);
    add("MAIN.Info_Feedback_ToMaster.Deliver_Force", kAdsLrealBytes, &feedback.deliver_force);
    add("MAIN.Info_Feedback_ToMaster.Rotate_Degree", kAdsLrealBytes, &feedback.rotate_degree);
    add("MAIN.Info_Feedback_ToMaster.Follow_Force", kAdsLrealBytes, &feedback.follow_force);
    for (std::size_t i = 0; i < kRobotPublishedAxisCount; ++i)
        add(kAxesPositionNames[i].c_str(), kAdsLrealBytes, &feedback.axes_pos[i]);

    const auto commonError = ReadDataBatch(requests.data(), requestCount, itemErrors.data());
    for (std::size_t i = 0; i < kRobotFeedbackLeafCount; ++i)
        feedbackErrors[i] = itemErrors[feedbackStart + i];

    std::size_t successfulItems = 0;
    for (std::size_t i = 0; i < requestCount; ++i) {
        if (itemErrors[i] == ADSERR_NOERR) {
            ++successfulItems;
        }
    }
    const auto failureDetails = FormatAdsReadFailures(requests, requestCount, itemErrors);
    if (!failureDetails.empty() && failureDetails != lastFailureDetails) {
        ROBOT_ERROR(true,
                    "Beckhoff Common leaf fields unavailable; publishing successful fields: "
                        << failureDetails)
    } else if (commonError == ADSERR_NOERR && !lastFailureDetails.empty()) {
        ROBOT_INFO(true, "Beckhoff Common leaf reads recovered; all fields are readable.")
    }
    lastFailureDetails = failureDetails;

    next.common_ads_error = commonError;
    KeepFirstError(next.overall_ads_error, commonError);
    next.move_state = itemErrors[0] == ADSERR_NOERR ? moveState : 0;
    next.prepare_state = itemErrors[1] == ADSERR_NOERR && prepareState == 1 ? 1 : 0;
    next.scope_type = itemErrors[2] == ADSERR_NOERR ? scopeType : 0;
    next.error_flags = 0;
    next.drive_errors = 0;
    next.motor_errors = 0;
    for (std::size_t i = 0; i < kMainMotorErrorCount; ++i) {
        if (itemErrors[3 + i] == ADSERR_NOERR && mainMotorErrors[i])
            next.motor_errors |= 1u << i;
    }
    ApplyRobotFeedback(feedback, feedbackErrors, next);
    MarkSnapshotGroup(next, SnapshotCommon, 0, successfulItems != 0, UnixNowNs());
}

/**
 * @brief 功能：读取 ERCP 在线、就绪、错误位、类型和运动状态并填充状态组。
 * @details 机制：按固定请求索引映射每个字段，只有对应读取成功时才设置语义位，随后标记 ERCP 状态组的有效性。
 */
std::uint32_t Beckhoff_Motor::PollErcpState(BeckhoffSnapshot &next,
                                            std::string &lastFailureDetails)
{
    bool online = false;
    bool ready = false;
    bool driveError = false;
    std::array<bool, kErcpDriveErrorCount> driveErrors{};
    bool motorError = false;
    std::array<bool, kErcpMotorErrorCount> motorErrors{};
    std::int32_t type = 0;
    std::int32_t moveStatus = 0;
    bool loadDirection = false;
    std::array<AdsReadRequest, kErcpStateRequestCount> requests{};
    std::size_t requestCount = 0;
    const auto add = [&](const char *name, unsigned long length, void *data) {
        requests[requestCount++] = {name, length, data};
    };
    add("MAIN.ERCP_Online_flag", kAdsBoolBytes, &online);
    add("POU_Ercp_CycleExecute.Ercp_Ready_State", kAdsBoolBytes, &ready);
    add("MAIN_ERCP.bErro_State_Drive_ERCP", kAdsBoolBytes, &driveError);
    for (std::size_t i = 0; i < kErcpDriveErrorCount; ++i)
        add(kErcpDriveErrorNames[i].c_str(), kAdsBoolBytes, &driveErrors[i]);
    add("MAIN_ERCP.bErro_State_Motor_ERCP", kAdsBoolBytes, &motorError);
    for (std::size_t i = 0; i < kErcpMotorErrorCount; ++i)
        add(kErcpMotorErrorNames[i].c_str(), kAdsBoolBytes, &motorErrors[i]);
    add("MAIN_ERCP.type_of_ERCP", kAdsInt32Bytes, &type);
    add("MAIN_ERCP.ERCP_Status_Feedback_ToMaster", kAdsInt32Bytes, &moveStatus);
    add("MAIN_ERCP.bERCP_Load_Exchange_Dir", kAdsBoolBytes, &loadDirection);

    std::array<std::uint32_t, requests.size()> errors{};
    const auto result = ReadDataBatch(requests.data(), requestCount, errors.data());
    const auto failureDetails = FormatAdsReadFailures(requests, requestCount, errors);
    if (!failureDetails.empty() && failureDetails != lastFailureDetails) {
        ROBOT_ERROR(true,
                    "Beckhoff ERCP state leaf fields unavailable; publishing successful fields: "
                        << failureDetails)
    } else if (result == ADSERR_NOERR && !lastFailureDetails.empty()) {
        ROBOT_INFO(true, "Beckhoff ERCP state leaf reads recovered; all fields are readable.")
    }
    lastFailureDetails = failureDetails;
    next.ercp_state_ads_error = result;
    KeepFirstError(next.overall_ads_error, result);
    next.ercp_flags = static_cast<std::uint16_t>(
        (errors[0] == ADSERR_NOERR && online ? 1u << 0 : 0u) |
        (errors[1] == ADSERR_NOERR && ready ? 1u << 1 : 0u) |
        (errors[2] == ADSERR_NOERR && driveError ? 1u << 3 : 0u) |
        (errors[kErcpLoadDirectionRequestIndex] == ADSERR_NOERR && loadDirection ? 1u << 2 : 0u) |
        (errors[kErcpMotorErrorFlagRequestIndex] == ADSERR_NOERR && motorError ? 1u << 4 : 0u));
    next.ercp_drive_errors = 0;
    for (std::size_t i = 0; i < kErcpDriveErrorCount; ++i) {
        if (errors[kErcpDriveErrorsRequestStart + i] == ADSERR_NOERR && driveErrors[i])
            next.ercp_drive_errors |= static_cast<std::uint16_t>(1u << i);
    }
    next.ercp_motor_errors = 0;
    for (std::size_t i = 0; i < kErcpMotorErrorCount; ++i) {
        if (errors[kErcpMotorErrorsRequestStart + i] == ADSERR_NOERR && motorErrors[i])
            next.ercp_motor_errors |= static_cast<std::uint16_t>(1u << i);
    }
    next.ercp_type = errors[kErcpTypeRequestIndex] == ADSERR_NOERR ? type : 0;
    next.ercp_move_status =
        errors[kErcpMoveStatusRequestIndex] == ADSERR_NOERR ? moveStatus : 0;
    MarkSnapshotGroup(next, SnapshotErcpState, 2, AnySuccessful(errors), UnixNowNs());
    return result;
}

/**
 * @brief 功能：读取 ERCP 力、位置、注入状态和球囊压力反馈。
 * @details 机制：一次批量提交 11 个叶字段，逐项按错误码选择真实值或安全零值，并更新 ERCP feedback 诊断组。
 */
std::uint32_t Beckhoff_Motor::PollErcpFeedback(BeckhoffSnapshot &next,
                                               std::string &lastFailureDetails)
{
    double deliverForce = 0;
    double guideWireForce = 0;
    double bowForce = 0;
    double deliverPosition = 0;
    double guideWirePosition = 0;
    double injectPosition01 = 0;
    double injectPosition02 = 0;
    std::int32_t injectState01 = 0;
    std::int32_t injectState02 = 0;
    std::int16_t balloonPressure = 0;
    double operatorPosition = 0;
    const std::array<AdsReadRequest, 11> requests{{
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Force", kAdsLrealBytes, &deliverForce},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.GuideWire_Force", kAdsLrealBytes, &guideWireForce},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Bow_Force", kAdsLrealBytes, &bowForce},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Pos", kAdsLrealBytes, &deliverPosition},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.GuideWire_Pos", kAdsLrealBytes, &guideWirePosition},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_CurPos_01", kAdsLrealBytes, &injectPosition01},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_CurPos_02", kAdsLrealBytes, &injectPosition02},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_State_01", kAdsInt32Bytes, &injectState01},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_State_02", kAdsInt32Bytes, &injectState02},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Balloon_Pressure", kAdsInt16Bytes, &balloonPressure},
        {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Operator_Pos", kAdsLrealBytes, &operatorPosition},
    }};
    std::array<std::uint32_t, requests.size()> errors{};
    const auto result = ReadDataBatch(requests.data(), requests.size(), errors.data());
    const auto failureDetails = FormatAdsReadFailures(requests, requests.size(), errors);
    if (!failureDetails.empty() && failureDetails != lastFailureDetails) {
        ROBOT_ERROR(true,
                    "Beckhoff ERCP feedback leaf fields unavailable; publishing successful fields: "
                        << failureDetails)
    } else if (result == ADSERR_NOERR && !lastFailureDetails.empty()) {
        ROBOT_INFO(true, "Beckhoff ERCP feedback leaf reads recovered; all fields are readable.")
    }
    lastFailureDetails = failureDetails;
    next.ercp_feedback_ads_error = result;
    KeepFirstError(next.overall_ads_error, result);
    next.ercp_deliver_force = errors[0] == ADSERR_NOERR ? deliverForce : 0;
    next.guide_wire_force = errors[1] == ADSERR_NOERR ? guideWireForce : 0;
    next.bow_force = errors[2] == ADSERR_NOERR ? bowForce : 0;
    next.ercp_deliver_position = errors[3] == ADSERR_NOERR ? deliverPosition : 0;
    next.guide_wire_position = errors[4] == ADSERR_NOERR ? guideWirePosition : 0;
    next.inject_current_position_01 = errors[5] == ADSERR_NOERR ? injectPosition01 : 0;
    next.inject_current_position_02 = errors[6] == ADSERR_NOERR ? injectPosition02 : 0;
    next.inject_state_01 = errors[7] == ADSERR_NOERR ? injectState01 : 0;
    next.inject_state_02 = errors[8] == ADSERR_NOERR ? injectState02 : 0;
    next.balloon_pressure = errors[9] == ADSERR_NOERR ? balloonPressure : 0;
    next.operator_position = errors[10] == ADSERR_NOERR ? operatorPosition : 0;
    MarkSnapshotGroup(next, SnapshotErcpFeedback, 3, AnySuccessful(errors), UnixNowNs());
    return result;
}

/**
 * @brief 功能：管理 ERCP 可选字段的探测、轮询和连续失败降级。
 * @details 机制：不可用时每秒探测一次；可用时同时读取状态/反馈，连续三轮失败后清除可选组并回到探测状态。
 */
void Beckhoff_Motor::PollErcpSnapshot(BeckhoffSnapshot &next,
                                      std::chrono::steady_clock::time_point cycleStarted,
                                      std::chrono::steady_clock::time_point &nextProbe,
                                      std::string &lastAvailabilityFailureDetails,
                                      std::string &lastStateFailureDetails,
                                      std::string &lastFeedbackFailureDetails)
{
    const auto readAvailability = [&](bool &readyProbe) {
        const auto result = ReadData(kErcpAvailabilitySymbol, kAdsBoolBytes, &readyProbe);
        const std::array<AdsReadRequest, 1> requests{{
            {kErcpAvailabilitySymbol, kAdsBoolBytes, &readyProbe},
        }};
        const std::array<std::uint32_t, 1> errors{{result}};
        const auto failureDetails = FormatAdsReadFailures(requests, requests.size(), errors);
        if (!failureDetails.empty() && failureDetails != lastAvailabilityFailureDetails) {
            ROBOT_ERROR(true,
                        "Beckhoff ERCP availability gate unavailable: " << failureDetails)
        } else if (failureDetails.empty() && !lastAvailabilityFailureDetails.empty()) {
            ROBOT_INFO(true,
                       "Beckhoff ERCP availability gate recovered; readiness symbol is readable.")
        }
        lastAvailabilityFailureDetails = failureDetails;
        return result == ADSERR_NOERR;
    };

    if (!m_ercp_available.load(std::memory_order_acquire) && cycleStarted >= nextProbe) {
        bool readyProbe = false;
        const bool detected = readAvailability(readyProbe);
        m_ercp_available.store(detected, std::memory_order_release);
        if (detected)
            m_ercp_failed_polls = 0;
        nextProbe = cycleStarted + std::chrono::seconds(1);
    }

    if (!m_ercp_available.load(std::memory_order_acquire)) {
        ClearOptionalErcpGroups(next);
        return;
    }

    const auto stateError = PollErcpState(next, lastStateFailureDetails);
    const auto feedbackError = PollErcpFeedback(next, lastFeedbackFailureDetails);
    if (stateError == ADSERR_NOERR && feedbackError == ADSERR_NOERR) {
        m_ercp_failed_polls = 0;
    } else {
        // Missing optional feedback leaves do not mean that the ERCP
        // interface disappeared. Re-check the readiness symbol and only
        // demote availability after repeated readiness read failures.
        bool readyProbe = false;
        const bool detected = readAvailability(readyProbe);
        if (detected) {
            m_ercp_failed_polls = 0;
        } else if (++m_ercp_failed_polls >= 3) {
            m_ercp_available.store(false, std::memory_order_release);
            m_ercp_failed_polls = 0;
            next.valid_groups &=
                static_cast<std::uint8_t>(~(SnapshotErcpState | SnapshotErcpFeedback));
            nextProbe = cycleStarted + std::chrono::seconds(1);
        }
    }
}

void Beckhoff_Motor::PublishSnapshot(BeckhoffSnapshot next)
{
    FinalizeSnapshotPoll(next, UnixNowNs());
    std::lock_guard<std::mutex> lock(m_snapshot_mutex);
    next.command_write_ads_error = m_snapshot.command_write_ads_error;
    m_snapshot = next;
}

// 状态轮询线程：按固定周期读取公共状态和 ERCP 状态，
// 汇总 ADS 诊断信息后发布一致的 Beckhoff 快照。
/**
 * @brief 功能：以 20 ms 周期调度公共状态、ERCP 状态并发布一致快照。
 * @details 机制：每轮复制上一快照并递增序号，按“读取—聚合诊断—发布—等待下一周期”顺序运行；线程中断用于生命周期退出。
 */
void Beckhoff_Motor::StateUpdateThread()
{
    auto nextErcpProbe = std::chrono::steady_clock::now();
    std::string lastCommonFailureDetails;
    std::string lastErcpAvailabilityFailureDetails;
    std::string lastErcpStateFailureDetails;
    std::string lastErcpFeedbackFailureDetails;

    while (!boost::this_thread::interruption_requested()) {
        // 阶段一：建立本轮快照基线和时间/序号元数据。
        const auto cycleStarted = std::chrono::steady_clock::now();
        BeckhoffSnapshot next = Snapshot();
        next.sequence += 1;
        next.poll_started_unix_ns = UnixNowNs();
        next.overall_ads_error = ADSERR_NOERR;

        // 阶段二：读取公共字段，再按可选能力读取 ERCP 字段并汇总错误。
        PollCommonSnapshot(next, lastCommonFailureDetails);
        PollErcpSnapshot(next,
                         cycleStarted,
                         nextErcpProbe,
                         lastErcpAvailabilityFailureDetails,
                         lastErcpStateFailureDetails,
                         lastErcpFeedbackFailureDetails);
        // 阶段三：一次性发布本轮快照，保持读侧看到的字段组相互对应。
        PublishSnapshot(next);

        std::this_thread::sleep_until(cycleStarted + std::chrono::milliseconds(20));
        boost::this_thread::interruption_point();
    }
}

}
} // namespace device::beckhoff
