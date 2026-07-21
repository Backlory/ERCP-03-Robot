/**
 * @file EndoScopy.h
 * @author ytom(杨涛)(yangtao1@sia.cn)
 * @brief 内镜运动学参数
 * @version 0.1
 * @date 2022-08-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __ENCOSCOPY__
#define __ENCOSCOPY__

#include <array>
#include <memory>
#include <Eigen/Dense>

namespace ercp {

    using namespace Eigen;

    /*************************************************/
    /* 调弯关系: wb+ <> b1, ws+ <> b4, wb- <> b3, ws- <> b2
     * wb = 大拨轮, ws = 小拨轮, b = beta
     * ┌─────────────────┐
     * │      ▲ b4       │
     * │      │ wb-      │
     * │b3◄───┼───►b1    │
     * │ws-   │  ws+/vx+ │
     * │      ▼ b2       │
     * │    wb+/vy+      │
     * └─────────────────┘
     */
    /*************************************************/
    class _EndoScopy {
    public:
        static const double DEG2RAD;

        template <typename T>
        static std::shared_ptr<_EndoScopy> Make()
        {
            static_assert(std::is_base_of<_EndoScopy, T>::value,
                "Target type must be derive from `_EndoScopy`.");
            return std::make_shared<T>();
        }

    public:
        /**
         * @brief 正运动学计算: 给定电机转速，计算目标运动速度
         *
         * @param w [Out]电机转速[°/s], [wb, ws], 范围[-1~1]
         * @param v 目标运动速度[°/s], [vx, vy]
         * @return true
         * @return false
         */
        bool ForwardKinematics(const Vector2d &w, Vector2d &v) const;

        /**
         * @brief 逆运动学解算: 给定目标运动速度，计算电机转速
         *
         * @param v 目标运动速度[°/s], [vx, vy]
         * @param w [Out]电机转速[°/s], [wb, ws], 范围[-1~1]
         * @return true
         * @return false
         */
        bool InverseKinematics(const Vector2d &v, Vector2d &w) const;

    public:
        // 内镜四根四的布局偏差角度[°]
        // 注意: 逆时针为正，顺时针顺序，分别以0，-90，180, 90方向为基准取绝对值
        const std::array<double, 4> beta;
        // 旋钮旋转速度到镜头转速的雅可比矩阵(由beta算出)
        // 注意: J.w^T = J.[wb+,ws+,wb-,ws-]^T = [vx+,vy+,vx-,vy-]^T = v^T
        const Matrix4d J, Jinv;

    protected:
        _EndoScopy(double b1, double b2, double b3, double b4);

        static Matrix4d CalcaJacobian(const double beta[4]);
    };

    //-------------------------------------------------------------------------

    class Auhua_AQ100 : public _EndoScopy {
    public:
        Auhua_AQ100()
            : _EndoScopy(12, 20, 12, 5)
        {
        }
    };

} // namespace ercp

#endif // __ROBOTBASE__
