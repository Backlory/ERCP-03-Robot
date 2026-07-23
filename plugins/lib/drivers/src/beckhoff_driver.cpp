#pragma once
#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <thread>
#include <iostream>
#include "utils.h"
#include "beckhoff_driver.hpp"

namespace device { namespace beckhoff {

    namespace {

        std::uint64_t UnixNowNs()
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        }

        void KeepFirstError(std::uint32_t &current, std::uint32_t candidate)
        {
            if (current == ADSERR_NOERR && candidate != ADSERR_NOERR) current = candidate;
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
    bool Beckhoff_Motor::OpenConn(string sIPAddr, int iPort)
    {
        if (IsOpen()) return true;
        CloseConn();

        {
            std::lock_guard<std::mutex> lock(m_snapshot_mutex);
            m_snapshot.connection_state = SnapshotConnectionState::Connecting;
            m_snapshot.published_unix_ns = UnixNowNs();
        }

        long nPort, nErr;
        USHORT nAdsState; //包含PLC的状态信�?
        // 倍福地址
        // AmsAddr bfAddr;// = BuildAddr(sIPAddr, iPort);

        {
            std::lock_guard<std::mutex> lock(m_ads_mutex);
            nPort = AdsPortOpen();
            m_port_open = nPort > 0;
        }
        if (!m_port_open) {
            std::lock_guard<std::mutex> lock(m_snapshot_mutex);
            m_snapshot.connection_state = SnapshotConnectionState::Disconnected;
            m_snapshot.overall_ads_error = ADSERR_CLIENT_PORTNOTOPEN;
            m_snapshot.published_unix_ns = UnixNowNs();
            return false;
        }
        BuildAddr(sIPAddr, iPort, m_Addr);
        //		nErr = AdsGetLocalAddress(&bfAddr);
        // if (nErr) {
        //	//printf(m_lastError, "Error: AdsGetLocalAddress: %d\n", nErr);
        //	return false;
        //}
        // bfAddr.port = 851;

        USHORT nDeviceState;
        {
            std::lock_guard<std::mutex> lock(m_ads_mutex);
            nErr = AdsSyncReadStateReq(&m_Addr, &nAdsState, &nDeviceState);
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
                nErr = AdsSyncWriteControlReq(&m_Addr, nAdsState, nDeviceState, 0, NULL);
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
        m_StateUpdate_Thread
            = boost::make_shared<boost::thread>(&Beckhoff_Motor::StateUpdateThread, this);
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
                nErr = AdsPortClose();
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

    // Motor的移�?
    //bool Beckhoff_Motor::MotorMove(double data[13], int iType, int bIsUpdate)
    //{
    //    MotorData d;
    //    // memcpy(d.MoveAngle, data, 13);
    //    for (int i = 0; i < 13; i++) {
    //        d.MoveAngle[i] = data[i];
    //    }
    //    d.ICtrlState = iType;
    //    d.IsUpdate = bIsUpdate;

    //    return WriteData((char *)"MAIN.Control_MotorMove", sizeof(d), &d);
    //}

    // 设置水气
    bool Beckhoff_Motor::LinearActuator(INT16 data[2])
    {
        return WriteData("MAIN.ILA_Node3", 4, data) == ADSERR_NOERR;
    }

    // 读取电机的位�?速度，电流等
    //bool Beckhoff_Motor::MotorPara(int iReadPos, double &dActPos, double &dActVel, int &iActCurrent)
    //{
    //    MotorParaData mpData[13];
    //    if (!ReadData((char *)"MAIN.MotorPara", sizeof(mpData), mpData))
    //        return false;

    //    dActPos = mpData[iReadPos].ActPos;
    //    dActVel = mpData[iReadPos].ActVel;
    //    iActCurrent = mpData[iReadPos].ActCurrent;

    //    return true;
    //}
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

    std::uint32_t Beckhoff_Motor::FollowOperationDataResult(
        unsigned long length, const void *data)
    {
        const std::uint32_t result = WriteData("MAIN.Follow_Control_Cmd", length, data);
        std::lock_guard<std::mutex> lock(m_snapshot_mutex);
        m_snapshot.command_write_ads_error = result;
        return result;
    }

    bool Beckhoff_Motor::FollowOperationData_Oneclick(double target, double bigAngle, double smlAngle, bool bSendState)
    {
        // 发送一键跟随状�?
        double pos[] = { target, bigAngle, smlAngle };

        bool bIsRe = WriteData("MAIN.FollowOper_Oneclick", sizeof(pos), pos) == ADSERR_NOERR;
        
        if (!bSendState)
            return bIsRe;
        else 
            return ArmOperation(BAO_FOLLOW_ONE);
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

    bool Beckhoff_Motor::SetEndoscopyType(int iType) {
        return WriteData("MAIN.type_of_scope", 4, &iType) == ADSERR_NOERR;
    }

    bool Beckhoff_Motor::EmergencyStop(bool bIsStop) {
        if (!bIsStop) {
            // 非急停为恢�?原有的运动状态恢复为初始状�?
            const auto moveState = MoveState();
            if (beckhoff_arm_move_state::BAMS_FOLDING == moveState ||
                beckhoff_arm_move_state::BAMS_OPENING == moveState ||
                beckhoff_arm_move_state::BAMS_FOLLOWING_ONE == moveState) {
                if ( !ArmOperation(beckhoff_arm_operation::BAO_NONE) ) return false;
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

    double Beckhoff_Motor::Follow_Length() { return Snapshot().common_values[0]; }

    ////2-3-2
    //double Beckhoff_Motor::SmallWhell() { return m_Data_Feedback.Asex_Pos[14]; } // Asex_Pos[14]; }
    //double Beckhoff_Motor::BigWhell() { return m_Data_Feedback.Asex_Pos[15]; } // Asex_Pos[15]; }
    
    // 2-3-3
    double Beckhoff_Motor::SmallWhell() { return Snapshot().common_values[17 + 16]; }
    double Beckhoff_Motor::BigWhell() { return Snapshot().common_values[17 + 17]; }

    double Beckhoff_Motor::SmallWhellCalc() { return Snapshot().common_values[2]; }
    double Beckhoff_Motor::BigWhellCalc() { return Snapshot().common_values[1]; }

    bool Beckhoff_Motor::Output_Switch(gpio_output_t out_switch) {
        const auto switches = Snapshot().output_switches;
        if (gpio_output_t::gas == out_switch ) return (switches & (1u << 1)) != 0;
        else if (gpio_output_t::water == out_switch) return (switches & (1u << 0)) != 0;
        else if (gpio_output_t::suct == out_switch) return (switches & (1u << 2)) != 0;

        return false;
    }

    double Beckhoff_Motor::Force(int iPos) {
        if (iPos < 0 || iPos > 9) return 0;
        return Snapshot().common_values[3 + iPos];
    }

    int Beckhoff_Motor::BatteryInfo() {
        return Snapshot().power_level;
    }

    // 获取阻力信息
    double Beckhoff_Motor::DeliverForce() {
        return Snapshot().common_values[14];
    }
    // 旋转角度
    double Beckhoff_Motor::RotateDegree() {
        return Snapshot().common_values[15];

    }
    // 抬钳�?
    double Beckhoff_Motor::Lifter() {
        return Snapshot().common_values[13];
    }

    //bool Beckhoff_Motor::MotorPara2(double dActPos[13], double dActVel[13], int iActCurrent[13])
    //{
    //    // MotorParaData mpData[13];
    //    // if (!ReadData((char*)"MAIN.MotorPara", sizeof(mpData), mpData)) return false;

    //    for (int i = 0; i < 13; i++) {
    //        dActPos[i] = m_Data_MotorPara[i].ActPos;
    //        dActVel[i] = m_Data_MotorPara[i].ActVel;
    //        iActCurrent[i] = m_Data_MotorPara[i].ActCurrent;
    //    }

    //    return true;
    //}

    ////2-3-2
    //bool Beckhoff_Motor::ReadAsexPos(double dAsex_Pos[21]) {
    //    for (int i = 0; i < 21; i++) {
    //        dAsex_Pos[i] = m_Data_Feedback.Asex_Pos[i];
    //    }
    //    return true;
    //}

    //2-3-3
    bool Beckhoff_Motor::ReadAsexPos(double dAsex_Pos[19])
    {
        const auto snapshot = Snapshot();
        for (int i = 0; i < 19; i++) {
            dAsex_Pos[i] = snapshot.common_values[17 + i];
        }
        return true;
    }
    // 读取编码
    bool Beckhoff_Motor::Encoder(INT32 data[5])
    {
        const auto snapshot = Snapshot();
        for (int i = 0; i < 5; i++) {
            data[i] = snapshot.encoders[i];
        }
        return true;
    }

    // 读取传感�?
    bool Beckhoff_Motor::Sensor(INT16 data[7])
    {
        const auto snapshot = Snapshot();
        for (int i = 0; i < 7; i++) {
            data[i] = snapshot.sensors[i];
        }
        return true;
    }

    bool Beckhoff_Motor::ERCPOperateState(bool state) // true = 操作中、False = 未操�?
    {
        return WriteData("MAIN_ERCP.bERCP_Operate_State_FromMaster", sizeof(state), &state)
            == ADSERR_NOERR;
    }

    bool Beckhoff_Motor::IsERCPOnline() { return (Snapshot().ercp_flags & (1u << 0)) != 0; }

    bool Beckhoff_Motor::IsERCPReady() { return (Snapshot().ercp_flags & (1u << 1)) != 0; }

    double Beckhoff_Motor::GetERCPDeliverForce() { return Snapshot().ercp_feedback[0]; }

    double Beckhoff_Motor::GetERCPGuidwireForce() { return Snapshot().ercp_feedback[1]; }

    double Beckhoff_Motor::GetERCPDeliverPos() { return Snapshot().ercp_feedback[8]; }

    // ================================================================================

    std::uint32_t Beckhoff_Motor::SymbolHandle(const char *paraName, unsigned long &handle)
    {
        const auto existing = m_symbol_handles.find(paraName);
        if (existing != m_symbol_handles.end()) {
            handle = existing->second;
            return ADSERR_NOERR;
        }

        if (!m_port_open) return ADSERR_CLIENT_PORTNOTOPEN;

        const auto nameLength = static_cast<unsigned long>(std::strlen(paraName) + 1);
        const auto result = static_cast<std::uint32_t>(AdsSyncReadWriteReq(&m_Addr,
            ADSIGRP_SYM_HNDBYNAME, 0, sizeof(handle), &handle, nameLength,
            const_cast<char *>(paraName)));
        if (result == ADSERR_NOERR) m_symbol_handles.emplace(paraName, handle);
        return result;
    }

    // 读取数据
    std::uint32_t Beckhoff_Motor::ReadData(
        const char *paraName, unsigned long length, void *data)
    {
        if (!IsOpen()) return ADSERR_CLIENT_PORTNOTOPEN;

        std::lock_guard<std::mutex> lock(m_ads_mutex);
        unsigned long handle = 0;
        const auto handleResult = SymbolHandle(paraName, handle);
        if (handleResult != ADSERR_NOERR) return handleResult;

        return static_cast<std::uint32_t>(
            AdsSyncReadReq(&m_Addr, ADSIGRP_SYM_VALBYHND, handle, length, data));
    }

    // 写入数据
    std::uint32_t Beckhoff_Motor::WriteData(
        const char *paraName, unsigned long length, const void *data)
    {
        if (!IsOpen()) return ADSERR_CLIENT_PORTNOTOPEN;

        std::lock_guard<std::mutex> lock(m_ads_mutex);
        unsigned long handle = 0;
        const auto handleResult = SymbolHandle(paraName, handle);
        if (handleResult != ADSERR_NOERR) return handleResult;

        return static_cast<std::uint32_t>(AdsSyncWriteReq(&m_Addr,
            ADSIGRP_SYM_VALBYHND, handle, length, const_cast<void *>(data)));
    }

    void Beckhoff_Motor::ReleaseSymbolHandles()
    {
        std::lock_guard<std::mutex> lock(m_ads_mutex);
        if (m_port_open) {
            for (const auto &entry : m_symbol_handles) {
                auto handle = entry.second;
                AdsSyncWriteReq(
                    &m_Addr, ADSIGRP_SYM_RELEASEHND, 0, sizeof(handle), &handle);
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
        while (!boost::this_thread::interruption_requested()) {
            const auto cycleStartedSteady = std::chrono::steady_clock::now();
            BeckhoffSnapshot next = Snapshot();
            next.sequence += 1;
            next.poll_started_unix_ns = UnixNowNs();
            next.overall_ads_error = ADSERR_NOERR;

            beckhoff_arm_move_state moveState{};
            FeedbackData feedback{};
            std::uint32_t commonError = ADSERR_NOERR;
            KeepFirstError(commonError, ReadData("MAIN.Status_Feedback_ToMaster",
                sizeof(moveState), &moveState));
            KeepFirstError(commonError, ReadData(
                "MAIN.Info_Feedback_ToMaster", sizeof(feedback), &feedback));
            next.common_ads_error = commonError;
            KeepFirstError(next.overall_ads_error, commonError);
            if (commonError == ADSERR_NOERR) {
                next.move_state = static_cast<std::uint32_t>(moveState);
                next.output_switches = static_cast<std::uint16_t>(
                    (feedback.Switch_Water ? 1u << 0 : 0u)
                    | (feedback.Switch_Gas ? 1u << 1 : 0u)
                    | (feedback.Switch_Suck ? 1u << 2 : 0u));
                next.power_level = feedback.Power_level;
                next.common_values[0] = feedback.Follow_Length;
                next.common_values[1] = feedback.Big_Whell;
                next.common_values[2] = feedback.Small_Whell;
                std::copy(std::begin(feedback.Force_Sensor), std::end(feedback.Force_Sensor),
                    next.common_values.begin() + 3);
                next.common_values[13] = feedback.lifter;
                next.common_values[14] = feedback.Deliver_force;
                next.common_values[15] = feedback.Rotate_Deqree;
                next.common_values[16] = feedback.Follow_Force;
                std::copy(std::begin(feedback.Asex_Pos), std::end(feedback.Asex_Pos),
                    next.common_values.begin() + 17);
                next.valid_groups |= SnapshotCommon;
                next.stale_groups &= static_cast<std::uint8_t>(~SnapshotCommon);
                next.sampled_at_unix_ns[0] = UnixNowNs();
            } else {
                next.stale_groups |= SnapshotCommon;
            }

            std::array<INT32, 5> encoders{};
            std::array<INT16, 7> sensors{};
            std::uint32_t rawIoError = ADSERR_NOERR;
            KeepFirstError(rawIoError,
                ReadData("MAIN.IEncoder", sizeof(encoders), encoders.data()));
            KeepFirstError(rawIoError,
                ReadData("MAIN.ISensor", sizeof(sensors), sensors.data()));
            next.raw_io_ads_error = rawIoError;
            KeepFirstError(next.overall_ads_error, rawIoError);
            if (rawIoError == ADSERR_NOERR) {
                std::copy(encoders.begin(), encoders.end(), next.encoders.begin());
                std::copy(sensors.begin(), sensors.end(), next.sensors.begin());
                next.valid_groups |= SnapshotRawIo;
                next.stale_groups &= static_cast<std::uint8_t>(~SnapshotRawIo);
                next.sampled_at_unix_ns[1] = UnixNowNs();
            } else {
                next.stale_groups |= SnapshotRawIo;
            }

            bool ercpOnline = false;
            bool ercpReady = false;
            bool driveError = false;
            bool driveErrors[14]{};
            bool motorError = false;
            bool motorErrors[12]{};
            int ercpType = 0;
            int ercpMoveStatus = 0;
            bool loadDirection = false;
            std::uint32_t ercpStateError = ADSERR_NOERR;
            KeepFirstError(ercpStateError, ReadData(
                "MAIN.ERCP_Online_flag", sizeof(ercpOnline), &ercpOnline));
            KeepFirstError(ercpStateError, ReadData(
                "POU_Ercp_CycleExecute.Ercp_Ready_State", sizeof(ercpReady), &ercpReady));
            KeepFirstError(ercpStateError, ReadData(
                "MAIN_ERCP.bErro_State_Drive_ERCP", sizeof(driveError), &driveError));
            KeepFirstError(ercpStateError, ReadData(
                "MAIN_ERCP.DriveErrorState_ERCP", sizeof(driveErrors), driveErrors));
            KeepFirstError(ercpStateError, ReadData(
                "MAIN_ERCP.bErro_State_Motor_ERCP", sizeof(motorError), &motorError));
            KeepFirstError(ercpStateError, ReadData(
                "MAIN_ERCP.MotorErrorState_ERCP", sizeof(motorErrors), motorErrors));
            KeepFirstError(ercpStateError,
                ReadData("MAIN_ERCP.type_of_ERCP", sizeof(ercpType), &ercpType));
            KeepFirstError(ercpStateError, ReadData(
                "MAIN_ERCP.ERCP_Status_Feedback_ToMaster",
                sizeof(ercpMoveStatus), &ercpMoveStatus));
            KeepFirstError(ercpStateError, ReadData(
                "MAIN_ERCP.bERCP_Load_Exchange_Dir", sizeof(loadDirection), &loadDirection));
            next.ercp_state_ads_error = ercpStateError;
            KeepFirstError(next.overall_ads_error, ercpStateError);
            if (ercpStateError == ADSERR_NOERR) {
                next.ercp_flags = static_cast<std::uint16_t>(
                    (ercpOnline ? 1u << 0 : 0u)
                    | (ercpReady ? 1u << 1 : 0u)
                    | (loadDirection ? 1u << 2 : 0u));
                next.ercp_drive_errors = 0;
                for (std::size_t i = 0; i < std::size(driveErrors); ++i) {
                    if (driveErrors[i]) next.ercp_drive_errors |= static_cast<std::uint16_t>(1u << i);
                }
                next.ercp_motor_errors = 0;
                for (std::size_t i = 0; i < std::size(motorErrors); ++i) {
                    if (motorErrors[i]) next.ercp_motor_errors |= static_cast<std::uint16_t>(1u << i);
                }
                next.ercp_type = ercpType;
                next.ercp_move_status = ercpMoveStatus;
                next.valid_groups |= SnapshotErcpState;
                next.stale_groups &= static_cast<std::uint8_t>(~SnapshotErcpState);
                next.sampled_at_unix_ns[2] = UnixNowNs();
            } else {
                next.stale_groups |= SnapshotErcpState;
            }

            ERCPFeedbackData ercpFeedback{};
            const auto ercpFeedbackError = ReadData("MAIN_ERCP.ERCP_Info_Feedback_ToMaster",
                sizeof(ercpFeedback), &ercpFeedback);
            next.ercp_feedback_ads_error = ercpFeedbackError;
            KeepFirstError(next.overall_ads_error, ercpFeedbackError);
            if (ercpFeedbackError == ADSERR_NOERR) {
                next.ercp_feedback = {ercpFeedback.Deliver_Force, ercpFeedback.GuideWire_Force,
                    ercpFeedback.Clamp_Force, ercpFeedback.Follow_Force_01,
                    ercpFeedback.Follow_Force_02, ercpFeedback.Bow_Force,
                    ercpFeedback.Inject_Force_01, ercpFeedback.Inject_Force_02,
                    ercpFeedback.Deliver_Pos, ercpFeedback.GuideWire_Pos,
                    ercpFeedback.Inject_CurPos_01, ercpFeedback.Inject_CurPos_02};
                next.inject_state_01 = ercpFeedback.Inject_State_01;
                next.inject_state_02 = ercpFeedback.Inject_State_02;
                next.valid_groups |= SnapshotErcpFeedback;
                next.stale_groups &= static_cast<std::uint8_t>(~SnapshotErcpFeedback);
                next.sampled_at_unix_ns[3] = UnixNowNs();
            } else {
                next.stale_groups |= SnapshotErcpFeedback;
            }

            next.poll_completed_unix_ns = UnixNowNs();
            next.published_unix_ns = next.poll_completed_unix_ns;
            if (next.overall_ads_error == ADSERR_NOERR) {
                next.consecutive_failed_polls = 0;
                next.connection_state = SnapshotConnectionState::Running;
            } else {
                if (next.consecutive_failed_polls
                    != (std::numeric_limits<std::uint32_t>::max)()) {
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

}} // namespace device::beckhoff
