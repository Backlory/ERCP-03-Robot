#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <vector>
#include <set>
#include <fmt/format.h>

#include "Device.hpp"
#include "beckhoff_snapshot.hpp"
#include "yunsbot_config.h"

#include <windows.h> //windowsAPI的函数声明和宏
#include "TcAdsDef.h" //结构体和常量的声明
#include "TcAdsAPI.h" // ADS函数的声明

namespace device { namespace beckhoff {
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

        struct FeedbackData {
            double Follow_Length; // 介入长度
            bool Switch_Water; // 水 状态
            bool Switch_Gas; // 气 状态
            bool Switch_Suck; // 吸引 状态
    
            double Big_Whell; // 大拨轮 0-1
            double Small_Whell; // 小拨轮 0-1

            double Force_Sensor[10]; // 力反馈信息
            INT16 Power_level; // 电池信息

            double lifter; // 抬钳器0-1;

            double Deliver_force; // 输送力

            double Rotate_Deqree; // 旋转角度

            double Follow_Force; //跟随力/N

            double Asex_Pos[19]; // 19个轴的信息
        };
        struct ERCPFeedbackData {
            double Deliver_Force; //器械输送力
            double GuideWire_Force; //导丝力
            double Clamp_Force; //夹紧力

            double Follow_Force_01; //跟随力01
            double Follow_Force_02; //跟随力02

            double Bow_Force; //拉弓力

            double Inject_Force_01; //注射器01力
            double Inject_Force_02; //注射器02力

            double Deliver_Pos; //器械输送位置
            double GuideWire_Pos; //导丝输送位置

            double Inject_CurPos_01; //注射器01位置 0，1
            double Inject_CurPos_02; //注射器02位置 0，1

            int Inject_State_01; //注射器01状态 0：待机 10：注射中 11：注射完成
            int Inject_State_02; //注射器02状态 0：待机 10：注射中 11：注射完成
        };

        // These types are ADS wire layouts. Keep them byte-for-byte compatible
        // with the production TwinCAT structures represented by beckhoff-GT.
        static_assert(sizeof(FeedbackData) == 304,
            "FeedbackData must match MAIN.Info_Feedback_ToMaster");
        static_assert(offsetof(FeedbackData, Follow_Length) == 0, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Switch_Water) == 8, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Switch_Gas) == 9, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Switch_Suck) == 10, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Big_Whell) == 16, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Small_Whell) == 24, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Force_Sensor) == 32, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Power_level) == 112, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, lifter) == 120, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Deliver_force) == 128, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Rotate_Deqree) == 136, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Follow_Force) == 144, "FeedbackData ABI drift");
        static_assert(offsetof(FeedbackData, Asex_Pos) == 152, "FeedbackData ABI drift");

        static_assert(sizeof(ERCPFeedbackData) == 104,
            "ERCPFeedbackData must match MAIN_ERCP.ERCP_Info_Feedback_ToMaster");
        static_assert(offsetof(ERCPFeedbackData, Deliver_Force) == 0,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, GuideWire_Force) == 8,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Clamp_Force) == 16,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Follow_Force_01) == 24,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Follow_Force_02) == 32,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Bow_Force) == 40,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Inject_Force_01) == 48,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Inject_Force_02) == 56,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Deliver_Pos) == 64,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, GuideWire_Pos) == 72,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Inject_CurPos_01) == 80,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Inject_CurPos_02) == 88,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Inject_State_01) == 96,
            "ERCPFeedbackData ABI drift");
        static_assert(offsetof(ERCPFeedbackData, Inject_State_02) == 100,
            "ERCPFeedbackData ABI drift");
    public:
        Beckhoff_Motor();
        ~Beckhoff_Motor();

        // 打开关闭链接
        bool OpenConn(string sIPAddr, int iPort);
        bool CloseConn();

        bool IsOpen() const { return m_bIsOpen.load(std::memory_order_acquire); }
        BeckhoffSnapshot Snapshot() const;

        // 获取信息
        bool ReadAsexPos(double dAsex_Pos[19]);

        bool Encoder(INT32 data[5]);
        bool Sensor(INT16 data[7]);
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
        std::uint32_t WriteData(const char *paraName, unsigned long length, const void *data);
        std::uint32_t SymbolHandle(const char *paraName, unsigned long &handle);
        void ReleaseSymbolHandles();

        // 创建地址
        bool BuildAddr(string sIP, int iPort, AmsAddr &bfAddr);

        // 更新状态线程
        void StateUpdateThread();
        boost::shared_ptr<boost::thread> m_StateUpdate_Thread;

        mutable std::mutex m_snapshot_mutex;
        BeckhoffSnapshot m_snapshot;

        mutable std::mutex m_ads_mutex;
        std::map<std::string, unsigned long> m_symbol_handles;

    private:
        // 是否打开连接
        std::atomic<bool> m_bIsOpen{false};
        bool m_port_open = false;

        // 倍福地址
        AmsAddr m_Addr;

    };

}} // namespace device::beckhoff
