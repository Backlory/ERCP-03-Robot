#pragma once
#include <Eigen/Dense>

using Vector5d = Eigen::Matrix<double, 5, 1>;

template <typename S>
Eigen::Matrix<S, 3, -1> Backproject(const Eigen::Matrix<S, 3, -1> &points,
    const Eigen::Matrix4d &intr, int width = -1, int height = -1)
{
    using Vector1 = Eigen::Matrix<S, 1, -1>;
    const Eigen::Matrix<S, 4, 4> K = intr.cast<S>();
    const Eigen::Matrix<S, 4, 4> IK = K.inverse();

    const Vector1 z = points.row(2);
    Eigen::Matrix<S, 3, -1> uv1 = points;
    uv1.row(2) = Vector1::Ones(points.cols());
    Eigen::Matrix<S, 3, -1> pix = IK.template block<3, 3>(0, 0) * uv1;

    if (width > 0)
        pix.row(0) *= 1.0 - 1.0 / width;
    if (height > 0)
        pix.row(1) *= 1.0 - 1.0 / height;

    pix.row(0) = pix.row(0).cwiseProduct(z);
    pix.row(1) = pix.row(1).cwiseProduct(z);
    pix.row(2) = pix.row(2).cwiseProduct(z);
    return pix;
}

template <typename S>
Eigen::Matrix<S, 2, -1> Project(const Eigen::Matrix<S, 3, -1> &points, int width, int height,
    const Eigen::Matrix4d &intr, bool normalized_intrinsics = true)
{
    using Vector1 = Eigen::Matrix<double, 1, -1>;
    const Vector1 z = points.row(2);
    if (!(z.array() > 0).all()) {
        throw std::runtime_error("Suppose depth for each pixel to be positive.");
    }

    const Vector1 iz = 1.0 / z.array();
    Eigen::Matrix<double, 2, 3> K = intr.template block<2, 3>(0, 0).cast<S>();
    K(0, 0) *= width / (width - 1.0);
    K(1, 1) *= height / (height - 1.0);

    Eigen::Matrix<S, 2, -1> uv = K * points;
    uv.row(0) = uv.row(0).cwiseProduct(iz);
    uv.row(1) = uv.row(1).cwiseProduct(iz);
    return uv;
}

//#include <torch/torch.h>

// torch::Tensor Backproject(const torch::Tensor &points, const torch::Tensor &intr, int width = -1,
//    int height = -1, bool inversed = false)
//{
//    if (points.dim() != 2 || intr.dim() != 2) {
//        throw std::runtime_error("Invalid dimension for `input` and `intr`.");
//    }
//    if (!(intr.size(0) == 4 && intr.size(1) == 4) || !(intr.size(0) == 3 && intr.size(1) == 3)) {
//        throw std::runtime_error("Invalid size of `intr`.");
//    }
//
//    auto IK = inversed ? intr : torch::inverse(intr);
//
//    IK();
//
//    const auto z = points.slice(0, 0, 1);
//    auto uv1 = points.clone();
//    points.slice(0, 2, 3) = 1.0;
//    auto pix = IK.slice() * uv1;
//
//    if (width > 0)
//        pix.row(0) *= 1.0 - 1.0 / width;
//    if (height > 0)
//        pix.row(1) *= 1.0 - 1.0 / height;
//
//    pix.row(0) = pix.row(0).cwiseProduct(z);
//    pix.row(1) = pix.row(1).cwiseProduct(z);
//    pix.row(2) = pix.row(2).cwiseProduct(z);
//    return pix;
//}

// torch::Tensor Project(const torch::Tensor &points, int width, int height, const torch::Tensor
// &intr,
//    bool normalized_intrinsics = true)
//{
//    using Vector1 = Eigen::Matrix<double, 1, -1>;
//    const Vector1 z = points.row(2);
//    if (!(z.array() > 0).all()) {
//        throw std::runtime_error("Suppose depth for each pixel to be positive.");
//    }
//
//    const Vector1 iz = 1.0 / z.array();
//    Eigen::Matrix<double, 2, 3> K = intr.block<2, 3>(0, 0).cast<S>();
//    K(0, 0) *= width / (width - 1.0);
//    K(1, 1) *= height / (height - 1.0);
//
//    Eigen::Matrix<S, 2, -1> uv = K * points;
//    uv.row(0) = uv.row(0).cwiseProduct(iz);
//    uv.row(1) = uv.row(1).cwiseProduct(iz);
//    return uv;
//}