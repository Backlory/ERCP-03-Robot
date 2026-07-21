//#include "cml_driver.hpp"
//#include "beckhoff_driver.hpp"
#include "serial/stepm_driver.hpp"

#include "robot_config.h"
#include "robot_devices.h"
#include "robot_settings.hpp"
#include "yunsbot_config.h"
#include "../YunSBot.h"
#include "RPC/ArmModule.hpp"
#include "../Kinematics/EndoScopy.h"
#include "../Kinematics/RobotBase.h"
#include "../Kinematics/RobotArm_v2.h"
namespace ercp {
    extern const double CUTTER_NEGLIM;
    extern const double CUTTER_POSLIM;
    extern const double CUTTER_SPEED_RATIO;
    extern const double STEP_SPEED;
    extern const double STEP_SPEED_RATIO;
    extern double GetFeedLength(bool disable_encoder, int encoder[5]);
    extern double GetFeedTotal();
    extern const Vector3d Trajectory(const double &length, const Vector3d &stored);
    extern const cml::JointMotorParam &GetJointConfig(motor_t mid);
    extern const std::map<motor_t, cml::JointMotorParam> JointConfigs_Auhua;
    extern const std::map<motor_t, cml::JointMotorParam> JointConfigs_AuhuaDeuodenum;
    extern task::paral_ptr<> BaseInitor();
    extern task::paral_ptr<> BaseDeInitor();

    robot_status GetRobotStatus()
    {
        auto &robot = GetRobot();

        auto &yunsbot = YunSBot::GetInstance();
        auto &armmod = rpc::ArmModule::GetInstance();

        robot_status status;
        status.time = ilsr::Time::wall_time();

        {
            static bool flag = true;
            status.robot_starting = yunsbot.base.IsRobotStarting();
            status.robot_stopping = yunsbot.base.IsRobotStopping();
            status.robot_running = yunsbot.base.IsRobotRunning();

            auto astate = armmod.get_current_state();
            auto plc_state = robot.BeckhoffArmMoveState();

            status.robot_folded = (plc_state == BAMS_FOLDED);
            status.robot_opened = (plc_state == BAMS_OPENED || plc_state == BAMS_FOLLOWING
                || plc_state == BAMS_FOLLOWED);

            //status.robot_folded
            //   = astate == rpc::arm_state_t::A3_Folded; //(plc_state == BAMS_FOLDED);
            //status.robot_opened
            //    = astate >= rpc::arm_state_t::A4_Opened; //(plc_state == BAMS_OPENED || plc_state ==
                                                         //BAMS_FOLLOWING
                //|| plc_state == BAMS_FOLLOWED);

            if (flag) {
                status.robot_following
                    = (plc_state == BAMS_FOLLOWING || plc_state == BAMS_FOLLOWED);
                flag = false;
            } else {
                status.robot_following = astate >= rpc::arm_state_t::A5_Following;
            }
            
        }

        {
            status._reserved[0] = robot.BeckhoffForce(0);
            status._reserved[1] = robot.BeckhoffForce(1); // robot.GetScopeForce(); // 输送力
            status._reserved[2] = robot.BeckhoffForce(2);
            status._reserved[3] = robot.BeckhoffForce(3);
            status._reserved[4] = robot.BeckhoffForce(4);
            status._reserved[5] = robot.BeckhoffForce(5);
            status._reserved[6] = robot.BeckhoffForce(6);
            status._reserved[7] = robot.BeckhoffForce(7);
            status._reserved[8] = robot.BeckhoffForce(8);
            status._reserved[9] = robot.BeckhoffForce(9);

            //2-3-3
            double asex_Pos[19];
            robot.BeckhoffReadAsexPos(asex_Pos);

            yunsbot.base.AddFLog(status._reserved[1], status._reserved[7], status._reserved[8],
                robot.BeckhoffFollowLength(), asex_Pos[5], asex_Pos[15], asex_Pos[16] );

        }

        {
            status._reserved2[0] = yunsbot.base.IsAutoMode();
            status._reserved2[1] = yunsbot.base.IsLogging();
        }


        {
            // auto q = robot.GetGpioOutput(gpio_output_t::gas);
            status.switch_gas = robot.Beckhoff_Switch(gpio_output_t::gas); //q && *q;
        }
        {
            //auto q = robot.GetGpioOutput(gpio_output_t::water);
            status.switch_water = robot.Beckhoff_Switch(gpio_output_t::water);
        }
        {
            //auto q = robot.GetGpioOutput(gpio_output_t::suct);
            status.switch_suct = robot.Beckhoff_Switch(gpio_output_t::suct);
        }

        // 注意: 下面的量非运行状态下不反馈
        if (!status.robot_running) {
            return status;
        }

        if (GetSettings().Device.Module.Arm()) {
            status.follow_length = robot.BeckhoffFollowLength();
        }

        // 2-3-3
        double asex_Pos[19];
        robot.BeckhoffReadAsexPos(asex_Pos);

        if (GetSettings().Device.Module.Operator()) {

            status._reserved[22] = robot.BeckhoffSmallWhellCalc();//asex_Pos[11] < 0 ? asex_Pos[11] / 140.0 : asex_Pos[11] / 80.0;  //robot.BeckhoffSmallWhellCalc(); 2-3-2 // 大小拨轮计算变量
            status._reserved[23] = robot.BeckhoffBigWhellCalc(); // asex_Pos[12] < 0 ? asex_Pos[12] / 90.0 : asex_Pos[12] / 140.0; // robot.BeckhoffBigWhellCalc(); // 2-3-2
            status.pos_bend_lr = -robot.BeckhoffSmallWhell();// -asex_Pos[11];// 2-2  // 2-3-1   -robot.BeckhoffSmallWhell();         //-asex_Pos[11];// 2-2
                //status.vel_bend_lr = current_vel[11];
            //}
            //{
            //    auto m = robot.GetMotor(motor_t::oper_big);
                status.pos_bend_ud = -robot.BeckhoffBigWhell();//-asex_Pos[12]; //2-2   // 2-3-1 -robot.BeckhoffBigWhell();//-asex_Pos[12]; 2-2
                //status.vel_bend_ud = current_vel[12];
            //}
            //{
            //    auto m = robot.GetMotor(motor_t::oper_rotate);
                status.pos_rotate = robot.BeckhoffRotateDegree();// asex_Pos[9]; // robot.BeckhoffRotateDegree();   2-3-1
                //asex_Pos[9];   2-2
                //status.vel_rotate = current_vel[9];
            //}
            //{
            //    auto m = robot.GetMotor(motor_t::oper_pincer);
                status.pos_pincer = robot.BeckhoffLifter(); // asex_Pos[18];
                //status.vel_pincer = current_vel[10];
            //}
        }

        // _reserved 0~7: force
        // _reserved 8~18: command

        int i = 0;
        int off = 19;

        if (GetSettings().Device.Module.Cannula()) {

            {
                //?????
                //auto &m = GetRobot().GetStepper();
                //double pos = 0;
                //m->GetPosition(pos);
                //status._reserved[off + (i++)] = pos;
            }
            {
                //?????
                //auto &m = GetRobot().GetPusher();
                //double pos = 0;
                //m->GetPosition(pos);
                //status._reserved[off + (i++)] = pos;
                //// auto &ctx = YunSBot::GetInstance().context;
                //// status._reserved[off + (i++)] = ctx.cannula_bowing_pos;
            }
        }

        i = 0;
        off = 22;

        if (GetSettings().Device.Module.Base()) {

            //??????????????????????auto m1 = robot.GetMotor(motor_t::base_1);
            //auto m2 = robot.GetMotor(motor_t::base_2);
            //auto m3 = robot.GetMotor(motor_t::base_3);
            //status._reserved[off + (i++)] = m1 ? m1->GetPosition() : 0;
            //status._reserved[off + (i++)] = m2 ? m2->GetPosition() : 0;
            //status._reserved[off + (i++)] = m3 ? m3->GetPosition() : 0;
        }

        i = 0;
        off = 25;

        if (GetSettings().Device.Module.Arm()) {
            // ?????????
            //auto m1 = robot.GetMotor(motor_t::arm_1);
            //auto m2 = robot.GetMotor(motor_t::arm_2);
            //auto m3 = robot.GetMotor(motor_t::arm_3);
            //auto m4 = robot.GetMotor(motor_t::arm_4);
            //status._reserved[off + (i++)] = asex_Pos[5];
            status._reserved[26] = asex_Pos[6]; // asex_Pos[12];     // -> cloud q2
            status._reserved[27] = asex_Pos[7];
            //asex_Pos[13]; // -> cloud q3
            status._reserved[28] = asex_Pos[8];
            //asex_Pos[14]; // -> cloud q4
        }

        i = 0;
        off = 29;

        if (GetSettings().Device.Module.Arm()) {
            // ?????????
            // auto m1 = robot.GetMotor(motor_t::arm_1);
            // auto m2 = robot.GetMotor(motor_t::arm_2);
            // auto m3 = robot.GetMotor(motor_t::arm_3);
            // auto m4 = robot.GetMotor(motor_t::arm_4);
            //status._reserved[off + (i++)] = asex_Pos[18];
            //status._reserved[off + (i++)] = asex_Pos[19]; // -> cloud q2
            status._reserved[off + (i++)] = robot.BeckhoffGetERCPDeliverPos();
            i++;
            status._reserved[off + (i++)] = 0; // -> cloud q3
            status._reserved[off + (i++)] = 0; // -> cloud q4
            status._reserved[off + (i++)] = 0; // -> cloud q4
            status._reserved[off + (i++)] = 0; // -> cloud q4
        }
        status._reserved[40] = robot.BeckhoffForce(1);
        status._reserved[41] = robot.BeckhoffGetERCPDeliverForce();
        return status;
    }

    std::string GetRobotStatusStr(robot_status &rstate)
    {
        auto &robot = GetRobot();
        std::string res = fmt::format("Time: {},\
            Pos_move: {},\
            Pos_bend_lr: {},\
            pos_bend_ud: {},\
            pos_rotate: {},\
            pos_pincer: {},\
            vel_move: {},\
            vel_bend_lr: {},\
            vel_bend_ud: {},\
            vel_rotate: {},\
            vel_pincer: {},\
            switch_water: {},\
            switch_gas: {},\
            switch_suct: {},\
            robot_starting: {},\
            robot_stopping: {},\
            robot_running: {},\
            robot_folded: {},\
            robot_opened: {},\
            robot_following: {},\
            follow_length: {},\
            _reserved: {},\
            _reserved2: {},\
            force_0: {},\
            force_1: {},\
            force_2: {},\
            force_3: {},\
            force_4: {},\
            force_5: {},\
            force_6: {},\
            force_7: {},\
            force_8: {},\
            force_9: {},\
            deliverForce: {},\
            guidewireForce: {}",
            rstate.time, rstate.pos_move, rstate.pos_bend_lr, rstate.pos_bend_ud, rstate.pos_rotate,
            rstate.pos_pincer, rstate.vel_move, rstate.vel_bend_lr, rstate.vel_bend_ud,
            rstate.vel_rotate, rstate.vel_pincer, rstate.switch_water, rstate.switch_gas,
            rstate.switch_suct, rstate.robot_starting, rstate.robot_stopping, rstate.robot_running,
            rstate.robot_folded, rstate.robot_opened, rstate.robot_following,
            rstate.follow_length,
            rstate._reserved[64],
            rstate._reserved2[32],
            robot.BeckhoffForce(0), robot.BeckhoffForce(1),
            robot.BeckhoffForce(2), robot.BeckhoffForce(3),
            robot.BeckhoffForce(4), robot.BeckhoffForce(5), robot.BeckhoffForce(6), 
            robot.BeckhoffForce(7), robot.BeckhoffForce(8), robot.BeckhoffForce(9), 
            robot.BeckhoffGetERCPDeliverForce(), robot.BeckhoffGetERCPGuidwireForce());
        return res;
    }

    void YunSBot::_base::ControlRunnable2(double t) {
        auto& robot = GetRobot();
        auto& yunsbot = YunSBot::GetInstance();
        control_cmd cmd;

        // 主端连接状态更新
        if (m_RobotAutoMode) {
            if (parent.master.GetMasterCmd(cmd, 0.1)) {
                if (cmd.switch_gas || cmd.switch_water || 0 != cmd.follow_comp_botton) {
                } else {
                    if (!parent.visual.GetVisionCmd(
                            cmd, 60.0)) { // 从端和云端电脑时间差要小于overtime(s)
                        m_RobotAutoMode = false;
                    }
                }
            } else {
                if (!parent.visual.GetVisionCmd(
                        cmd, 60.0)) { // 从端和云端电脑时间差要小于overtime(s)
                    m_RobotAutoMode = false;
                }
            }
        }
        else {
            if (!parent.master.GetMasterCmd(cmd, 0.1)) {
            }
        }

        ////////////////////////////////////////////////
        // Stage II. Send status by UDP(~0.020s)
        ////////////////////////////////////////////////
        const auto rstate = GetRobotStatus();

        //if (rstate.robot_following) {
        //    robot.BeckhoffFollowData(sizeof(cmd), &cmd);
        //}

        // 发送基座控制信息
        
        //robot.BeckhoffBaseMove(sizeof(cmd.base), &cmd.base);

        //写入是否操作ERCP
        bool ercp_online = robot.BeckhoffIsERCPOnline();
        bool ercp_ready = robot.BeckhoffIsERCPReady();
        static bool bERCPOnline = ercp_online;
        static bool bERCPReady = false;
        if (ercp_online != bERCPOnline) {
            if (!ercp_online)
                robot.BeckhoffERCPOperateState(false);
            else if (ercp_ready)
                robot.BeckhoffERCPOperateState(true);
        } else if (ercp_online && ercp_ready != bERCPReady) {
            robot.BeckhoffERCPOperateState(ercp_ready);
        }
        bERCPOnline = ercp_online;
        bERCPReady = ercp_ready;

        // 发送跟随信息
        beckhoff_follow_cmd follow_cmd;
        build_follow_cmd(cmd, follow_cmd);
        robot.BeckhoffFollowData(sizeof(follow_cmd), &follow_cmd);
    }

    // 建立跟随命令信息
    void YunSBot::_base::build_follow_cmd(const control_cmd& cmd, beckhoff_follow_cmd& follow_cmd) {
        follow_cmd.follow_comp_botton = isnan(cmd.follow_comp_botton) ? 0 : cmd.follow_comp_botton;
        
        follow_cmd.vel_move = isnan(cmd.vel_move) ? 0 : cmd.vel_move;
        follow_cmd.vel_rotate = isnan(cmd.vel_rotate) ? 0 : cmd.vel_rotate;
        follow_cmd.vel_pincer = isnan(cmd.vel_pincer) ? 0 : cmd.vel_pincer;
        follow_cmd.vel_bend_lr = isnan(cmd.vel_bend_lr) ? 0 : cmd.vel_bend_lr;
        follow_cmd.vel_bend_ud = isnan(cmd.vel_bend_ud) ? 0 : cmd.vel_bend_ud;

        // ERCP操作器
        follow_cmd.vel_cutter_rot
            = isnan(cmd.cannula.vel_cutter_rotate) ? 0 : cmd.cannula.vel_cutter_rotate;
        follow_cmd.vel_cutter_bend
            = isnan(cmd.cannula.vel_cutter_bend) ? 0 : cmd.cannula.vel_cutter_bend;
        follow_cmd.vel_wire_feed 
            = isnan(cmd.cannula.vel_wire_feed) ? 0 : cmd.cannula.vel_wire_feed;
        follow_cmd.vel_cutter_feed
            = isnan(cmd.cannula.vel_cutter_move) ? 0 : cmd.cannula.vel_cutter_move;

        //follow_cmd.follow_comp_botton = cmd.follow_comp_botton;
        //follow_cmd.vel_move = cmd.vel_move;
        //follow_cmd.vel_rotate = cmd.vel_rotate;
        //follow_cmd.vel_pincer =  cmd.vel_pincer;
        //follow_cmd.vel_bend_lr = cmd.vel_bend_lr;
        //follow_cmd.vel_bend_ud = cmd.vel_bend_ud;

        //// ERCP操作器
        //follow_cmd.vel_cutter_rot
        //    =  cmd.cannula.vel_cutter_rotate;
        //follow_cmd.vel_cutter_bend
        //    = cmd.cannula.vel_cutter_bend;
        //follow_cmd.vel_wire_feed = cmd.cannula.vel_wire_feed;
        //follow_cmd.vel_cutter_feed
        //    =  cmd.cannula.vel_cutter_move;

        follow_cmd.home_rotate = cmd.home_rotate;
        follow_cmd.home_bend_lr = cmd.home_bend_lr;
        follow_cmd.home_bend_ud = cmd.home_bend_ud;

        follow_cmd.switch_water = cmd.switch_water;
        follow_cmd.switch_gas = cmd.switch_gas;
        follow_cmd.switch_suct = cmd.switch_suct;
    }


    bool  YunSBot::_base::IsChangeModel() {
        auto& robot = GetRobot();
        auto& yunsbot = YunSBot::GetInstance();
        control_cmd cmd;

        // 初始化状态，可以进行移动
        if (beckhoff_arm_move_state::BAMS_START == robot.BeckhoffArmMoveState()) return true;

        // 主端连接状态更新
        if (m_RobotAutoMode) {
            if (!parent.visual.GetVisionCmd(cmd, 1.0)) {
                m_RobotAutoMode = false;
            }
        }else {
            if (!parent.master.GetMasterCmd(cmd, 0.1)) {
            }
        }

        if (cmd.base.use_base) {
            return false;
        }

        return true;
    }



} // namespace ercp
