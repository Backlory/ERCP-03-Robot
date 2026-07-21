#pragma once

#include <boost/thread/thread.hpp>

class VirtualRobot {

public:
    static VirtualRobot *get_instance() { return VirtualRobot::m_instance; }

    virtual void run();
    virtual void close();
    virtual bool is_online() const;

public:
    //********************反馈********************//
    virtual double get_bend_ud();
    virtual double get_bend_lr();
    virtual double get_rotation();
    virtual double get_torque();
    virtual double get_length();
    virtual double get_q2();
    virtual double get_q3();
    virtual double get_q4();
    virtual bool is_forwarding();

protected:
    //********************约束********************//
    typedef struct {
        double x, y;
    } direct_t;

    /// <summary>
    /// 工作空间约束
    /// </summary>
    /// <param name="operation">当前操作行为</param>
    /// <param name="isfoward">当前是否前进</param>
    /// <param name="force">当前输送力大小</param>
    /// <param name="image_quality">当前图像质量</param>
    /// <param name="dire">当前目标控制运动方向</param>
    /// <returns></returns>
    virtual bool VirtualRobot::ws_constraints(int operation, int isfoward, double force,
        double image_quality, const VirtualRobot::direct_t &dire);

protected:
    VirtualRobot();
    VirtualRobot(const VirtualRobot &) = delete;

protected:
    //********************控制流********************//
    virtual int runner();

    double m_ftime = -1;
    boost::shared_mutex m_access;
    std::shared_ptr<boost::thread> m_thread = nullptr;

protected:
    static VirtualRobot *m_instance;
};
