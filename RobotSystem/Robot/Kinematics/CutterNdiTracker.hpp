#ifndef _CUTTER_NDI_TRACKER_
#    define _CUTTER_NDI_TRACKER_

#    pragma once

#    include <assert.h>
#    include <algorithm>
#    include <map>
#    include <vector>
#    include <cmath>
#    include <cmath>
#    include <memory>
#    include <numeric>

#    include <Eigen/Dense>
#    include "utils.h"

using namespace Eigen;

class CutterNdiTracker {
public:
    /**
     * @brief 一阶有限逆向数值差分系数,参数是差分点数,取值2~6
     */
    static const std::map<int, std::vector<double>> Coefficients;
    // static const std::vector<double>                MeanCoefficients;

    /**
     * @brief 计算一阶有限逆向数值差分
     *
     * @param data 数据系列，小于差分点数则尝试采用更少点数的差分系数计算
     * @param order 差分点数
     * @return double
     */
    static double numdiff(const std::vector<double> &data, const int &order = 5) {
        assert(order >= 2 && order <= 6);
        if (data.size() >= 2) {
            int ord = std::min(order, (int)data.size());
            if (Coefficients.find(ord) == Coefficients.end()) { assert(false); }
            auto coeff = Coefficients.at(ord);
            return std::inner_product(data.end() - ord, data.end(), coeff.begin(), 0.0);
        } else {
            return 0;
        }
    }

    /**
     * @brief 计算滑动平均，即计算序列最后winsize个的均值
     *
     * @param data 数据序列，小于窗口大小的序列自动计算其均值
     * @param winsize 窗口大小
     * @return double
     */
    static double moveavg(const std::vector<double> &data, const int &winsize = 5) {
        assert(winsize >= 1);
        if (data.size() >= 1) {
            int ws = std::min(winsize, (int)data.size());
            return std::accumulate(data.end() - ws, data.end(), 0.0) / ws;
        } else {
            return 0;
        }
    }

public:
    typedef enum : int {
        Unready = 0,
        Finish  = Unready,
        Ready,
        Running,
    } State_t;

    typedef enum : int {
        EstPositive = 1,
        EstNegative = -EstPositive,
        EstStop     = 0,
    } EstDire_t;

    typedef struct Boundary_t {
        double       upper = 0, lower = 0;
        const double threshold;
        Boundary_t(double u, double l, double thre = 0.02) : upper(u), lower(l), threshold(thre) {
            assert(u >= l);
        }
    };

private:
    /// 初始误差记录
    double residual0 = 0;
    /// 上一次误差记录，在控制方向变换时更新
    double residual_ex = 0;
    /// 上一次控制量记录，每个迭代周期更新
    double control_ex = 0;
    /// 跟踪器状态机状态
    State_t state = Unready;
    /// 预测运动方向
    EstDire_t estim_direction = EstStop;
    /// 预测运动方向的运动计数
    size_t count_direction = 0;
    /// 方向切换计数
    size_t count_direction_change = 0;

    /// 一个方向最小持续运动周期，报纸能测试出有效的误差
    int min_last_cycles = 0;
    /// 结束迭代条件：连续stop_condition区间内有足够比例的递减，且最近一次出现局部最小回升的情况
    int stop_condition = 0;
    /// 结束迭代的比例阈值
    const double stop_threshold;
    /// 陷入停滞判定条件是：连续reverse_condition个周期误差增长速度为0，认为陷入不运动状态
    int trap_condition = 0;
    /// 判定运动停滞的阈值
    const double trap_threshold;
    /// 初始化测试方向是否反向
    const bool init_reversed = false;
    /// 当误差增量不小于valid_residul_increase认为有效
    double valid_residul_increase = 0;

    /// 历史误差数据记录
    std::vector<double> residual_history;
    /// 历史误差数据一阶微分
    std::vector<double> residual_history_1rd;
    /// 历史误差数据一阶微分平滑处理（滤波）
    std::vector<double> residual_history_smooth;

    /// 切开刀主端位置边界范围
    std::shared_ptr<Boundary_t> boundary = nullptr;

    bool global_start = true;

public:
    /**
     * @brief 构造函数
     * @param initrev 初始预测是否反向，详见 @ref init_reversed
     * @param stopthre 陷入停滞判定阈值，详见 @ref stop_threshold
     * @param trapthre 陷入停滞判定阈值，详见 @ref trap_threshold
     */
    CutterNdiTracker(bool initrev = false, double stopthre = 0.80, double trapthre = 1e-4)
        : init_reversed(initrev), stop_threshold(stopthre), trap_threshold(trapthre) {}
    /**
     * @brief 构造函数
     * @param bound 主端控制位置边界
     * @param initrev 初始预测是否反向，详见 @ref init_reversed
     * @param stopthre 陷入停滞判定阈值，详见 @ref stop_threshold
     * @param trapthre 陷入停滞判定阈值，详见 @ref trap_threshold
     */
    CutterNdiTracker(Boundary_t bound, bool initrev = false, double stopthre = 0.80, double trapthre = 0.005)
        : CutterNdiTracker(initrev, stopthre, trapthre) {
        boundary = std::make_shared<Boundary_t>(bound);
    }
    CutterNdiTracker(const CutterNdiTracker &) = delete;

public:
    /**
     * @brief 初始化追踪器
     *
     * @param stopcond 停止条件数，详见 @ref stop_condition
     * @param trapcond 停滞判定条件数，详见 @ref trap_condition
     * @param minlast 最小持续周期，详见 @ref min_last_cycles
     * @param valid_dresidual 有效误差增量，详见 @ref valid_residul_increase
     * @note 0.14 ~= 8deg
     */
    void Init(int stopcond = 20, int trapcond = 10, int minlast = 10, double valid_dresidual = 0.01) {

        stop_condition         = stopcond;
        trap_condition         = trapcond;
        min_last_cycles        = minlast;
        valid_residul_increase = valid_dresidual;
        assert(trap_threshold > 0);
        assert(min_last_cycles > 0);
        assert(valid_residul_increase > 0);

        state       = Ready;
        control_ex  = 0;
        residual_ex = residual0 = 0;
        count_direction         = 0;
        count_direction_change  = 0;

        residual_history.clear();
        residual_history_1rd.clear();
        residual_history_smooth.clear();
    }

    /**
     * @brief 追踪器迭代更新过程
     *
     * @param dq 方向误差四元数
     * @param position 当前主端位置控制量
     * @param endposition 当前反馈从端位置
     * @param iskeep 是否保持之前的控制量，用于控制条件无效的默认情况
     * @return double
     */
    double Update(const Quaterniond &dq, const double &position, const Vector3d &endposition, bool iskeep = true) {

        double control = iskeep ? control_ex : 0;
        /* 这里zpos和ztar是四元数形式的单位向量，若记作：zpos=(0, vz1), ztar=(0, vz2)，则
         * dq = zpos * ztar = (-vz1·vz2, vz1×vz2) 分别是点乘和叉乘
         * 第一项可以表示两个方向的夹角cos大小，第二项（向量）可以表示夹角的转轴方向和夹角sin大小 */
        // double residual = 1.0 - dq.GetW();  // 控制策略，优化：zpos和ztar的夹角
        double residual = acos(dq.w());

        // 保存历史误差
        residual_history.push_back(residual);
        // 计算和保存一阶近似微分
        double diff_residual = numdiff(residual_history);
        residual_history_1rd.push_back(diff_residual);
        // 计算和保存一阶近似微分的滑动平均
        double smooth_residual = moveavg(residual_history_1rd);
        residual_history_smooth.push_back(smooth_residual);

        // TODO: 是否加一个判断位置是否有效变化?，以应对错误导致的长时间停止

        if (state == Ready) {
            if (global_start) {
                /** 根据当前状态给定试探的初始控制方向(只在第一次起作用)
                 *  试探依据： 夹角转轴方向向量分量和(能保证不同转向时，值的符号不同) **/
                estim_direction = ((!init_reversed) && (dq.x() + dq.y() + dq.z() > 0)) ? EstPositive : EstNegative;
                global_start    = false;
            }
            // 如果从边界附近开始则向着边界外则反向
            if (boundary != nullptr) {
                double err = std::abs(boundary->upper - boundary->lower) * boundary->threshold;
                if (std::abs(position - boundary->lower) <= err && estim_direction == EstNegative) {
                    estim_direction = (EstDire_t)-estim_direction;
                } else if (std::abs(position - boundary->upper) <= err && estim_direction == EstPositive) {
                    estim_direction = (EstDire_t)-estim_direction;
                }
            }
            residual_ex = residual0 = residual;
            state                   = Running;
        } else if (state == Running) {
            bool isfinish = false;
            {  // 退出迭代判定
                // 结束迭代条件：连续stop_condition区间内有足够比例的递减，且最近一次出现局部最小回升的情况
                // TODO:是否需要序列过长也认为失败停止?考虑加入与初始误差的比较条件?
                if (stop_condition > 1 && residual_history_smooth.size() >= stop_condition) {
                    // 临近区间的最小值
                    auto imin = std::min_element(residual_history.end() - stop_condition, residual_history.end());
                    // 最近末端大于局部最小值
                    if (residual_history.back() > *imin) {
                        size_t minus = 0;
                        for (auto ite = residual_history_smooth.end() - stop_condition;
                             ite != residual_history_smooth.end(); ++ite) {
                            minus += (int)(*ite < -DBL_EPSILON);
                        }
                        isfinish = (minus >= (int)(stop_condition * stop_threshold));
                    }
                }
            }

            if (isfinish) {
                state = Finish;
                // TRACE2("Get local roughs.\n");
                // TODO: 记录正确经验
            } else {

                bool istrapped = true;
                {  // 长期停滞判定
                    if (trap_condition > 1 && residual_history_smooth.size() >= trap_condition) {
                        auto ite = residual_history_smooth.end() - trap_condition;
                        for (; ite != residual_history_smooth.end(); ++ite) {
                            istrapped = istrapped && (std::abs(*ite) < trap_threshold);
                        }
                    } else {
                        istrapped = false;
                    }
                }

                bool isatbound = false;
                {  // 到达边界判定
                    if (boundary != nullptr) {
                        double err = std::abs(boundary->upper - boundary->lower) * boundary->threshold;
                        if (std::abs(position - boundary->lower) <= err ||
                            std::abs(position - boundary->upper) <= err) {
                            isatbound = true;
                        }
                    }
                }

                // 要求1：相比于初始时，残差增量较大
                count_direction++;

                // TODO:已经反向但误差仍增长?
                // TODO:约束不超出边界运动

                // 误差比初始大
                if (count_direction >= min_last_cycles) {
                    // 误差比起始时增长，且比上次切换方向时增长，反向之后再切换方向的要求更高
                    // TODO：这个增强条件的机制需要进一步考虑
                    if (residual - residual0 > valid_residul_increase * (1.0 + 0.2 * count_direction_change)) {
                        // 切换方向相对误差应当增长，防止突然误差增大时的振荡
                        if (residual - residual_ex > valid_residul_increase) {
                            estim_direction = (EstDire_t)-estim_direction;
                            residual_ex     = residual;
                            count_direction = 0;
                            count_direction_change++;
                        }
                    }
                }

                // 频繁切换方向则结束
                if (state != Finish && count_direction_change > 4) {  // TODO:参数化
                    state = Finish;
                    TRACE2("Frequently change direction.\n");
                }

                // 误差在边界处停滞则结束,TODO:如何优化?
                if (state != Finish && istrapped && isatbound) {
                    state = Finish;
                    // TRACE2("Trapped in boundary.\n");
                }
            }
        }

        if (state >= Ready) {
            control_ex = control = estim_direction * 1.0;
        } else {  // Unready and so on
            control_ex = control = 0;
        }

        return control;
    }

    /**
     * @brief 追踪器迭代更新过程，用于控制间隙默认保持控制量
     *
     * @return double
     */
    double Update() {
        double control = control_ex;
        if (state <= CutterNdiTracker::Finish) { control_ex = control = 0; }
        return control;
    }

    /**
     * @brief 禁用跟踪器
     */
    void Disable() {
        state = Unready;
    }

    /**
     * @brief 设置边界
     * @note 输入NULL会清除边界值
     */
    void SetBoundary(Boundary_t *bound = NULL) {
        boundary = bound ? std::make_shared<Boundary_t>(bound->upper, bound->lower) : nullptr;
    }

    /**
     * @brief 获取当前误差值
     *
     * @return double
     */
    double GetResidual() {
        return residual_history.back();
    }

    /**
     * @brief 获取当前误差序列：误差、原始一阶微分、平滑异界微分
     *
     * @return std::vector<double>
     */
    std::vector<double> GetResiduals() {
        return {
            residual_history.back(),
            residual_history_1rd.back(),
            residual_history_smooth.back(),
        };
    }

    /**
     * @brief 获取跟踪器当前运行状态
     *
     * @return const State_t
     */
    const State_t GetState() const {
        return state;
    }
};

const std::map<int, std::vector<double>> CutterNdiTracker::Coefficients = {
    {2, {-1.0, 1.0}},
    {3, {1 / 2.0, -2.0, 3 / 2.0}},
    {4, {-1 / 3.0, 3 / 2.0, -3.0, 11 / 6.0}},
    {5, {1 / 4.0, -4 / 3.0, 3.0, -4.0, 25 / 12.0}},
    {6, {1 / 5.0, -5 / 4.0, 10 / 3.0, -5.0, 5.0, -137 / 60.0}}};

// const std::vector<double> MeanCoefficients = {
//    0, 0, 0, 0.1, 0.15, 0.25, 0.5
//}

#endif  // _Cutter_Ndi_Tracker_
