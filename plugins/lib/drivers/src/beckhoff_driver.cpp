#pragma once
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <iostream>
#include "robot_config.h"
#include "utils.h"
#include "beckhoff_driver.hpp"

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

} // namespace

// =============================================================================
// 倍福电机对象

Beckhoff_Motor::Beckhoff_Motor() = default;
Beckhoff_Motor::~Beckhoff_Motor()
{
    CloseConn();
}

// 打开链接
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
    if (nAdsState == 6) {
        nAdsState = ADSSTATE_RUN;
        {
            std::lock_guard<std::mutex> lock(m_ads_mutex);
            nErr = AdsWriteControl(nAdsState, nDeviceState, 0, nullptr);
        }
        if (nErr) {
            // printf(m_lastError, "Error: AdsSyncWriteControlReq: ", nErr);
            {
                std::lock_guard<std::mutex> lock(m_snapshot_mutex);
                m_snapshot.overall_ads_error = static_cast<std::uint32_t>(nErr);
            }
            CloseConn();
            return false;
        }
    }
    m_bIsOpen.store(true, std::memory_order_release);
    m_sum_read_supported = true;
    m_common_block_read_enabled = ValidateRobotFeedbackLayout();
    // MAIN_ERCP is optional. Probe a gold-standard leaf symbol instead of
    // assuming an ABI for the enclosing TwinCAT STRUCT.
    double ercpProbe = 0;
    m_ercp_available.store(ReadData("MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Force",
                                    sizeof(ercpProbe),
                                    &ercpProbe) == ADSERR_NOERR,
                           std::memory_order_release);
    m_StateUpdate_Thread =
        boost::make_shared<boost::thread>(&Beckhoff_Motor::StateUpdateThread, this);
    return true;
}

// 关闭链接
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

bool Beckhoff_Motor::EmergencyStop(bool bIsStop)
{
    if (!bIsStop) {
        // 非急停为恢�?原有的运动状态恢复为初始状�?
        const auto moveState = MoveState();
        if (beckhoff_arm_move_state::BAMS_FOLDING == moveState ||
            beckhoff_arm_move_state::BAMS_OPENING == moveState) {
            if (!ArmOperation(beckhoff_arm_operation::BAO_NONE))
                return false;
        }
    }

    int iStop = bIsStop ? 1 : 0;
    return WriteData("MAIN.Emergency_Stop_FromMaster", 4, &iStop) == ADSERR_NOERR;
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

bool Beckhoff_Motor::Output_Switch(gpio_output_t out_switch)
{
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

std::uint32_t Beckhoff_Motor::QuerySymbolInfo(const char *paraName, AdsSymbolInfo &info)
{
    info = {};
    if (!IsOpen())
        return ADSERR_CLIENT_PORTNOTOPEN;

    std::array<std::uint8_t, 4096> buffer{};
    std::uint32_t bytesRead = 0;
    const auto nameLength = static_cast<unsigned long>(std::strlen(paraName) + 1);
    std::lock_guard<std::mutex> lock(m_ads_mutex);
    if (!m_port_open)
        return ADSERR_CLIENT_PORTNOTOPEN;

    const auto result = AdsReadWrite(ADSIGRP_SYM_INFOBYNAMEEX,
                                     0,
                                     static_cast<unsigned long>(buffer.size()),
                                     buffer.data(),
                                     nameLength,
                                     paraName,
                                     &bytesRead);
    if (result != ADSERR_NOERR)
        return result;
    if (bytesRead < sizeof(AdsSymbolEntry))
        return ADSERR_DEVICE_INVALIDSIZE;

    AdsSymbolEntry entry{};
    std::memcpy(&entry, buffer.data(), sizeof(entry));
    info.index_group = static_cast<std::uint32_t>(entry.iGroup);
    info.index_offset = static_cast<std::uint32_t>(entry.iOffs);
    info.size = static_cast<std::uint32_t>(entry.size);
    return ADSERR_NOERR;
}

std::uint32_t Beckhoff_Motor::QuerySymbolSize(const char *paraName, std::uint32_t &size)
{
    AdsSymbolInfo info;
    const auto result = QuerySymbolInfo(paraName, info);
    size = info.size;
    return result;
}

bool Beckhoff_Motor::ValidateSymbolSize(const char *paraName, std::size_t expectedSize)
{
    std::uint32_t onlineSize = 0;
    const auto result = QuerySymbolSize(paraName, onlineSize);
    if (result != ADSERR_NOERR) {
        ROBOT_ERROR(true,
                    "Cannot validate Beckhoff symbol "
                        << paraName << " size; ADS error=0x" << std::uppercase << std::hex << result
                        << std::nouppercase << std::dec << ". Falling back to leaf reads.")
        return false;
    }
    if (onlineSize != expectedSize) {
        ROBOT_ERROR(true,
                    "Beckhoff symbol " << paraName << " size mismatch: online=" << onlineSize
                                       << " bytes, expected=" << expectedSize
                                       << " bytes. Falling back to leaf reads.")
        return false;
    }

    ROBOT_INFO(true, "Beckhoff symbol " << paraName << " validated at " << onlineSize << " bytes.")
    return true;
}

bool Beckhoff_Motor::ValidateRobotFeedbackLayout()
{
    constexpr const char *parentName = "MAIN.Info_Feedback_ToMaster";
    if (!ValidateSymbolSize(parentName, RobotFeedbackBlockSize))
        return false;

    AdsSymbolInfo parent;
    const auto parentResult = QuerySymbolInfo(parentName, parent);
    if (parentResult != ADSERR_NOERR)
        return false;

    struct ExpectedField {
        const char *suffix;
        std::uint32_t offset;
        std::uint32_t size;
    };
    constexpr std::array<ExpectedField, 13> fields{{
        {"Follow_Length", 0, 8},
        {"Switch_Water", 8, 1},
        {"Switch_Gas", 9, 1},
        {"Switch_Suck", 10, 1},
        {"Axes_Pos", 16, 21 * 8},
        {"Big_Wheel", 184, 8},
        {"Small_Wheel", 192, 8},
        {"Force_Sensor", 200, 10 * 8},
        {"Power_level", 280, 2},
        {"lifter", 288, 8},
        {"Deliver_Force", 296, 8},
        {"Rotate_Degree", 304, 8},
        {"Follow_Force", 312, 8},
    }};

    for (const auto &field : fields) {
        const auto fieldName = std::string(parentName) + "." + field.suffix;
        AdsSymbolInfo online;
        const auto result = QuerySymbolInfo(fieldName.c_str(), online);
        if (result != ADSERR_NOERR) {
            ROBOT_ERROR(true,
                        "Cannot validate Beckhoff feedback field "
                            << fieldName << "; ADS error=0x" << std::uppercase << std::hex << result
                            << std::nouppercase << std::dec << ". Falling back to leaf reads.")
            return false;
        }
        const bool offsetIsValid = online.index_group == parent.index_group &&
                                   online.index_offset >= parent.index_offset &&
                                   online.index_offset - parent.index_offset == field.offset;
        if (!offsetIsValid || online.size != field.size) {
            ROBOT_ERROR(true,
                        "Beckhoff feedback ABI mismatch at "
                            << fieldName << ": online offset="
                            << (online.index_offset >= parent.index_offset
                                    ? online.index_offset - parent.index_offset
                                    : online.index_offset)
                            << ", size=" << online.size << "; expected offset=" << field.offset
                            << ", size=" << field.size << ". Falling back to leaf reads.")
            return false;
        }
    }

    ROBOT_INFO(true, "Beckhoff feedback ABI validated: 320 bytes and all 13 field offsets match.")
    return true;
}

std::uint32_t Beckhoff_Motor::SymbolHandle(const char *paraName, unsigned long &handle)
{
    const auto existing = m_symbol_handles.find(paraName);
    if (existing != m_symbol_handles.end()) {
        handle = existing->second;
        return ADSERR_NOERR;
    }

    if (!m_port_open)
        return ADSERR_CLIENT_PORTNOTOPEN;

    const auto nameLength = static_cast<unsigned long>(std::strlen(paraName) + 1);
    const auto result =
        AdsReadWrite(ADSIGRP_SYM_HNDBYNAME, 0, sizeof(handle), &handle, nameLength, paraName);
    if (result == ADSERR_NOERR)
        m_symbol_handles.emplace(paraName, handle);
    return result;
}

std::uint32_t Beckhoff_Motor::ReadDataBatch(const AdsReadRequest *requests,
                                            std::size_t count,
                                            std::uint32_t *itemErrors)
{
    if (requests == nullptr || itemErrors == nullptr || count == 0)
        return ADSERR_DEVICE_INVALIDPARM;

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
    std::vector<std::uint32_t> sumRequest(count * 3, 0);
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
            sumRequest[i * 3] = ADSIGRP_SYM_VALBYHND;
            sumRequest[i * 3 + 1] = static_cast<std::uint32_t>(handles[i]);
            sumRequest[i * 3 + 2] = static_cast<std::uint32_t>(requests[i].length);
            payloadSize += requests[i].length;
        }
        if (firstError != ADSERR_NOERR)
            return firstError;

        const auto resultBytes = count * sizeof(std::uint32_t) + payloadSize;
        std::vector<std::uint8_t> sumResult(resultBytes, 0);
        const auto result =
            AdsReadWrite(ADSIGRP_SUMUP_READ,
                         static_cast<unsigned long>(count),
                         static_cast<unsigned long>(sumResult.size()),
                         sumResult.data(),
                         static_cast<unsigned long>(sumRequest.size() * sizeof(std::uint32_t)),
                         sumRequest.data());
        if (result == ADSERR_DEVICE_SRVNOTSUPP || result == ADSERR_DEVICE_INVALIDGRP) {
            m_sum_read_supported = false;
            fallBackToSequential = true;
        } else if (result != ADSERR_NOERR) {
            std::fill_n(itemErrors, count, result);
            return result;
        } else {
            std::size_t dataOffset = count * sizeof(std::uint32_t);
            firstError = ADSERR_NOERR;
            for (std::size_t i = 0; i < count; ++i) {
                std::memcpy(&itemErrors[i],
                            sumResult.data() + i * sizeof(std::uint32_t),
                            sizeof(itemErrors[i]));
                if (itemErrors[i] == ADSERR_NOERR) {
                    std::memcpy(requests[i].data,
                                sumResult.data() + dataOffset,
                                requests[i].length);
                } else {
                    AdsWrite(ADSIGRP_SYM_RELEASEHND, 0, sizeof(handles[i]), &handles[i]);
                    m_symbol_handles.erase(requests[i].name);
                }
                KeepFirstError(firstError, itemErrors[i]);
                dataOffset += requests[i].length;
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
}

// 建立地址
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

// 更新状态线程函�?
void Beckhoff_Motor::StateUpdateThread()
{
    auto nextErcpProbe = std::chrono::steady_clock::now();
    std::string lastCommonFailureDetails;
    while (!boost::this_thread::interruption_requested()) {
        const auto cycleStartedSteady = std::chrono::steady_clock::now();
        BeckhoffSnapshot next = Snapshot();
        next.sequence += 1;
        next.poll_started_unix_ns = UnixNowNs();
        next.overall_ads_error = ADSERR_NOERR;

        beckhoff_arm_move_state moveState{};
        INT16 prepareState{};
        std::int32_t scopeType{};
        FeedbackData feedback{};
        std::array<std::uint8_t, RobotFeedbackBlockSize> feedbackBlock{};
        bool mainMotorErrors[19]{};
        std::uint32_t commonError = ADSERR_NOERR;
        std::ostringstream commonFailures;
        bool hasCommonFailure = false;
        std::array<AdsReadRequest, 17> commonRequests{};
        std::array<std::uint32_t, 17> commonItemErrors{};
        std::size_t commonRequestCount = 0;
        const auto addCommon = [&](const char *name, unsigned long length, void *data) {
            commonRequests[commonRequestCount++] = {name, length, data};
        };
        addCommon("MAIN.Status_Feedback_ToMaster", sizeof(moveState), &moveState);
        addCommon("MAIN.iPrepare_State", sizeof(prepareState), &prepareState);
        addCommon("MAIN.type_of_scope", sizeof(scopeType), &scopeType);
        // These three symbols are absent from the deployed robot-body PLC.
        // Keep their wire fields at zero instead of making the whole Common
        // snapshot stale and suppressing Robot status publication.
        addCommon("MAIN.MotorErrorState", sizeof(mainMotorErrors), mainMotorErrors);
        if (m_common_block_read_enabled) {
            addCommon("MAIN.Info_Feedback_ToMaster",
                      static_cast<unsigned long>(feedbackBlock.size()),
                      feedbackBlock.data());
        } else {
#define ADD_MAIN_FEEDBACK(field, value)                                                            \
    addCommon("MAIN.Info_Feedback_ToMaster." field, sizeof(value), &(value))
            ADD_MAIN_FEEDBACK("Follow_Length", feedback.Follow_Length);
            ADD_MAIN_FEEDBACK("Switch_Water", feedback.Switch_Water);
            ADD_MAIN_FEEDBACK("Switch_Gas", feedback.Switch_Gas);
            ADD_MAIN_FEEDBACK("Switch_Suck", feedback.Switch_Suck);
            ADD_MAIN_FEEDBACK("Big_Wheel", feedback.Big_Whell);
            ADD_MAIN_FEEDBACK("Small_Wheel", feedback.Small_Whell);
            addCommon("MAIN.Info_Feedback_ToMaster.Force_Sensor",
                      sizeof(feedback.Force_Sensor),
                      feedback.Force_Sensor);
            ADD_MAIN_FEEDBACK("Power_level", feedback.Power_level);
            ADD_MAIN_FEEDBACK("lifter", feedback.lifter);
            ADD_MAIN_FEEDBACK("Deliver_Force", feedback.Deliver_force);
            ADD_MAIN_FEEDBACK("Rotate_Degree", feedback.Rotate_Deqree);
            ADD_MAIN_FEEDBACK("Follow_Force", feedback.Follow_Force);
            addCommon("MAIN.Info_Feedback_ToMaster.Axes_Pos",
                      sizeof(feedback.Axes_Pos),
                      feedback.Axes_Pos);
#undef ADD_MAIN_FEEDBACK
        }
        commonError =
            ReadDataBatch(commonRequests.data(), commonRequestCount, commonItemErrors.data());
        if (m_common_block_read_enabled &&
            commonItemErrors[commonRequestCount - 1] != ADSERR_NOERR) {
            m_common_block_read_enabled = false;
            ROBOT_ERROR(true,
                        "Beckhoff Common block read failed after startup validation; "
                        "disabling block reads until reconnect and falling back to leaf reads.")
        }
        for (std::size_t i = 0; i < commonRequestCount; ++i) {
            const auto error = commonItemErrors[i];
            if (error == ADSERR_NOERR)
                continue;
            if (hasCommonFailure)
                commonFailures << ", ";
            commonFailures << commonRequests[i].name << "=0x" << std::uppercase << std::hex << error
                           << std::nouppercase << std::dec;
            if (error == ADSERR_DEVICE_SYMBOLNOTFOUND) {
                commonFailures << "(ADS_SYMBOL_NOT_FOUND)";
            }
            hasCommonFailure = true;
        }
        if (commonError == ADSERR_NOERR && m_common_block_read_enabled &&
            !DecodeRobotFeedbackBlock(feedbackBlock.data(), feedbackBlock.size(), feedback)) {
            commonError = ADSERR_DEVICE_INVALIDSIZE;
            commonFailures << "MAIN.Info_Feedback_ToMaster=0x" << std::uppercase << std::hex
                           << commonError << std::nouppercase << std::dec << "(INVALID_BLOCK_SIZE)";
            hasCommonFailure = true;
        }
        const auto commonFailureDetails = commonFailures.str();
        if (commonError != ADSERR_NOERR && commonFailureDetails != lastCommonFailureDetails) {
            ROBOT_ERROR(true,
                        "Beckhoff Common status publication blocked; failed required fields: "
                            << commonFailureDetails)
        } else if (commonError == ADSERR_NOERR && !lastCommonFailureDetails.empty()) {
            ROBOT_INFO(true, "Beckhoff Common status recovered; all required fields are readable.")
        }
        lastCommonFailureDetails = commonFailureDetails;
        next.common_ads_error = commonError;
        KeepFirstError(next.overall_ads_error, commonError);
        if (commonError == ADSERR_NOERR) {
            next.move_state = static_cast<std::uint32_t>(moveState);
            ApplyRobotFeedback(feedback, next);
            next.prepare_state = prepareState == 1 ? 1 : 0;
            next.scope_type = scopeType;
            next.error_flags = 0;
            next.drive_errors = 0;
            next.motor_errors = 0;
            for (std::size_t i = 0; i < std::size(mainMotorErrors); ++i)
                if (mainMotorErrors[i])
                    next.motor_errors |= 1u << i;
            next.valid_groups |= SnapshotCommon;
            next.stale_groups &= static_cast<std::uint8_t>(~SnapshotCommon);
            next.sampled_at_unix_ns[0] = UnixNowNs();
        } else {
            next.stale_groups |= SnapshotCommon;
        }

        if (!m_ercp_available.load(std::memory_order_acquire) &&
            cycleStartedSteady >= nextErcpProbe) {
            double ercpProbe = 0;
            const bool detected =
                ReadData("MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Force",
                         sizeof(ercpProbe),
                         &ercpProbe) == ADSERR_NOERR;
            m_ercp_available.store(detected, std::memory_order_release);
            if (detected)
                m_ercp_failed_polls = 0;
            nextErcpProbe = cycleStartedSteady + std::chrono::seconds(1);
        }

        if (m_ercp_available.load(std::memory_order_acquire)) {
            bool ercpOnline = false;
            bool ercpReady = false;
            bool driveError = false;
            bool driveErrors[13]{};
            bool motorError = false;
            bool motorErrors[11]{};
            int ercpType = 0;
            int ercpMoveStatus = 0;
            bool loadDirection = false;
            const std::array<AdsReadRequest, 9> ercpStateRequests{{
                {"MAIN.ERCP_Online_flag", sizeof(ercpOnline), &ercpOnline},
                {"POU_Ercp_CycleExecute.Ercp_Ready_State", sizeof(ercpReady), &ercpReady},
                {"MAIN_ERCP.bErro_State_Drive_ERCP", sizeof(driveError), &driveError},
                {"MAIN_ERCP.DriveErrorState_ERCP", sizeof(driveErrors), driveErrors},
                {"MAIN_ERCP.bErro_State_Motor_ERCP", sizeof(motorError), &motorError},
                {"MAIN_ERCP.MotorErrorState_ERCP", sizeof(motorErrors), motorErrors},
                {"MAIN_ERCP.type_of_ERCP", sizeof(ercpType), &ercpType},
                {"MAIN_ERCP.ERCP_Status_Feedback_ToMaster",
                 sizeof(ercpMoveStatus),
                 &ercpMoveStatus},
                {"MAIN_ERCP.bERCP_Load_Exchange_Dir", sizeof(loadDirection), &loadDirection},
            }};
            std::array<std::uint32_t, ercpStateRequests.size()> ercpStateItemErrors{};
            const auto ercpStateError = ReadDataBatch(ercpStateRequests.data(),
                                                      ercpStateRequests.size(),
                                                      ercpStateItemErrors.data());
            next.ercp_state_ads_error = ercpStateError;
            KeepFirstError(next.overall_ads_error, ercpStateError);
            if (ercpStateError == ADSERR_NOERR) {
                next.ercp_flags = static_cast<std::uint16_t>(
                    (ercpOnline ? 1u << 0 : 0u) | (ercpReady ? 1u << 1 : 0u) |
                    (loadDirection ? 1u << 2 : 0u) | (driveError ? 1u << 3 : 0u) |
                    (motorError ? 1u << 4 : 0u));
                next.ercp_drive_errors = 0;
                for (std::size_t i = 0; i < std::size(driveErrors); ++i) {
                    if (driveErrors[i])
                        next.ercp_drive_errors |= static_cast<std::uint16_t>(1u << i);
                }
                next.ercp_motor_errors = 0;
                for (std::size_t i = 0; i < std::size(motorErrors); ++i) {
                    if (motorErrors[i])
                        next.ercp_motor_errors |= static_cast<std::uint16_t>(1u << i);
                }
                next.ercp_type = ercpType;
                next.ercp_move_status = ercpMoveStatus;
                next.valid_groups |= SnapshotErcpState;
                next.stale_groups &= static_cast<std::uint8_t>(~SnapshotErcpState);
                next.sampled_at_unix_ns[2] = UnixNowNs();
            } else {
                next.stale_groups |= SnapshotErcpState;
            }

            double ercpDeliverForce = 0;
            double guideWireForce = 0;
            double bowForce = 0;
            double ercpDeliverPosition = 0;
            double guideWirePosition = 0;
            double injectPosition01 = 0;
            double injectPosition02 = 0;
            std::int32_t injectState01 = 0;
            std::int32_t injectState02 = 0;
            std::int16_t balloonPressure = 0;
            double operatorPosition = 0;
            const std::array<AdsReadRequest, 11> ercpFeedbackRequests{{
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Force",
                 sizeof(ercpDeliverForce),
                 &ercpDeliverForce},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.GuideWire_Force",
                 sizeof(guideWireForce),
                 &guideWireForce},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Bow_Force", sizeof(bowForce), &bowForce},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.ERCP_Deliver_Pos",
                 sizeof(ercpDeliverPosition),
                 &ercpDeliverPosition},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.GuideWire_Pos",
                 sizeof(guideWirePosition),
                 &guideWirePosition},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_CurPos_01",
                 sizeof(injectPosition01),
                 &injectPosition01},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_CurPos_02",
                 sizeof(injectPosition02),
                 &injectPosition02},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_State_01",
                 sizeof(injectState01),
                 &injectState01},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Inject_State_02",
                 sizeof(injectState02),
                 &injectState02},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Balloon_Pressure",
                 sizeof(balloonPressure),
                 &balloonPressure},
                {"MAIN_ERCP.ERCP_Info_Feedback_ToMaster.Operator_Pos",
                 sizeof(operatorPosition),
                 &operatorPosition},
            }};
            std::array<std::uint32_t, ercpFeedbackRequests.size()> ercpFeedbackItemErrors{};
            const auto ercpFeedbackError = ReadDataBatch(ercpFeedbackRequests.data(),
                                                         ercpFeedbackRequests.size(),
                                                         ercpFeedbackItemErrors.data());
            next.ercp_feedback_ads_error = ercpFeedbackError;
            KeepFirstError(next.overall_ads_error, ercpFeedbackError);
            if (ercpFeedbackError == ADSERR_NOERR) {
                next.ercp_deliver_force = ercpDeliverForce;
                next.guide_wire_force = guideWireForce;
                next.bow_force = bowForce;
                next.ercp_deliver_position = ercpDeliverPosition;
                next.guide_wire_position = guideWirePosition;
                next.inject_current_position_01 = injectPosition01;
                next.inject_current_position_02 = injectPosition02;
                next.inject_state_01 = injectState01;
                next.inject_state_02 = injectState02;
                next.balloon_pressure = balloonPressure;
                next.operator_position = operatorPosition;
                next.valid_groups |= SnapshotErcpFeedback;
                next.stale_groups &= static_cast<std::uint8_t>(~SnapshotErcpFeedback);
                next.sampled_at_unix_ns[3] = UnixNowNs();
            } else {
                next.stale_groups |= SnapshotErcpFeedback;
            }

            if (ercpStateError == ADSERR_NOERR && ercpFeedbackError == ADSERR_NOERR) {
                m_ercp_failed_polls = 0;
            } else if (++m_ercp_failed_polls >= 3) {
                m_ercp_available.store(false, std::memory_order_release);
                m_ercp_failed_polls = 0;
                next.valid_groups &=
                    static_cast<std::uint8_t>(~(SnapshotErcpState | SnapshotErcpFeedback));
                nextErcpProbe = cycleStartedSteady + std::chrono::seconds(1);
            }
        } else {
            next.ercp_state_ads_error = ADSERR_NOERR;
            next.ercp_feedback_ads_error = ADSERR_NOERR;
            next.ercp_flags = 0;
            next.ercp_drive_errors = 0;
            next.ercp_motor_errors = 0;
            next.valid_groups &=
                static_cast<std::uint8_t>(~(SnapshotErcpState | SnapshotErcpFeedback));
            next.stale_groups &=
                static_cast<std::uint8_t>(~(SnapshotErcpState | SnapshotErcpFeedback));
            next.sampled_at_unix_ns[2] = 0;
            next.sampled_at_unix_ns[3] = 0;
        }

        next.poll_completed_unix_ns = UnixNowNs();
        next.published_unix_ns = next.poll_completed_unix_ns;
        if (next.overall_ads_error == ADSERR_NOERR) {
            next.consecutive_failed_polls = 0;
            next.connection_state = SnapshotConnectionState::Running;
        } else {
            if (next.consecutive_failed_polls != (std::numeric_limits<std::uint32_t>::max)()) {
                next.consecutive_failed_polls += 1;
            }
            next.connection_state = SnapshotConnectionState::Degraded;
        }

        {
            std::lock_guard<std::mutex> lock(m_snapshot_mutex);
            next.command_write_ads_error = m_snapshot.command_write_ads_error;
            m_snapshot = next;
        }

        const auto nextCycle = cycleStartedSteady + std::chrono::milliseconds(20);
        std::this_thread::sleep_until(nextCycle);
        boost::this_thread::interruption_point();
    }
}

}
} // namespace device::beckhoff
