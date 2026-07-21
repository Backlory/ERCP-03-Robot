/**
 * @file _EndoScopy.cpp
 * @author ytom(杨涛)(yangtao1@sia.cn)
 * @brief 内镜运动学参数
 * @version 0.1
 * @date 2022-08-19
 *
 * @copyright Copyright (c) 2022
 *
 */

#define _USE_MATH_DEFINES
#include <float.h>
#include <cmath>
#include <type_traits>
#include "EndoScopy.h"

namespace ercp {

    const double _EndoScopy::DEG2RAD = (0.017453292519943295);

    _EndoScopy::_EndoScopy(double b1, double b2, double b3, double b4)
        : beta({ b1, b2, b3, b4 })
        , J(CalcaJacobian(beta.data()))
        , Jinv(J.inverse())
    {
    }

    Matrix4d _EndoScopy::CalcaJacobian(const double beta[4])
    {
        // clang-format off
        Matrix4d J = (Matrix4d() << 
            // -> ws+ wb+ ws- wb-
            //         b1                            b2                            b3                            b4
            + std::cos(beta[0]*DEG2RAD),  + std::sin(beta[1]*DEG2RAD),  0*std::cos(beta[2]*DEG2RAD),  + std::sin(beta[3]*DEG2RAD), // vx+
            + std::sin(beta[0]*DEG2RAD),  + std::cos(beta[1]*DEG2RAD),  0*std::sin(beta[2]*DEG2RAD),  0*std::cos(beta[3]*DEG2RAD), // vy+
            0*std::cos(beta[0]*DEG2RAD),  0*std::sin(beta[1]*DEG2RAD),  + std::cos(beta[2]*DEG2RAD),  0*std::sin(beta[3]*DEG2RAD), // vx-
            0*std::sin(beta[0]*DEG2RAD),  0*std::cos(beta[1]*DEG2RAD),  + std::sin(beta[2]*DEG2RAD),  + std::cos(beta[3]*DEG2RAD)  // vy-
        ).finished();
        // clang-format on
        return J;
    }

    bool _EndoScopy::ForwardKinematics(const Vector2d &w, Vector2d &v) const
    {
        Vector4d W = Vector4d::Zero();
        W(0) = std::abs(std::max(w(1), 0.0)); // ws+
        W(1) = std::abs(std::max(w(0), 0.0)); // wb+
        W(2) = std::abs(std::min(w(1), 0.0)); // ws-
        W(3) = std::abs(std::min(w(0), 0.0)); // wb-
        Vector4d V = J * W;
        v(0) = V(0) - V(2); // vx
        v(1) = V(1) - V(3); // vy
        return true;
    }

    bool _EndoScopy::InverseKinematics(const Vector2d &v, Vector2d &w) const
    {
        Vector4d V = Vector4d::Zero();
        V(0) = std::abs(std::max(v(0), 0.0)); // vx+
        V(1) = std::abs(std::max(v(1), 0.0)); // vy+
        V(2) = std::abs(std::min(v(0), 0.0)); // vx-
        V(3) = std::abs(std::min(v(1), 0.0)); // vy-
        Vector4d W = Jinv * V;
        w(0) = W(1) - W(3); // wb
        w(1) = W(0) - W(2); // ws
        return true;
    }

} // namespace ercp
