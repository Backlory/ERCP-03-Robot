#pragma once
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <torch/torch.h>

namespace F = torch::nn::functional;

namespace utils {

    torch::Tensor toTensor(const cv::Mat &arr, bool cuda = false)
    {
        auto device = (cuda && torch::cuda::is_available()) ? c10::kCUDA : c10::kCPU;
        switch (arr.type()) {
        case CV_8UC1: {
            cv::Mat floatArr;
            arr.convertTo(floatArr, CV_32FC1, 1 / 255.0);
            return torch::tensor(torch::ArrayRef<float>(floatArr.ptr<float>(),
                                     floatArr.rows * floatArr.cols * floatArr.channels()),
                c10::TensorOptions().device(device))
                .reshape({ 1, 1, arr.rows, arr.cols })
                .contiguous();
        }
        case CV_8UC3: {
            cv::Mat floatArr;
            arr.convertTo(floatArr, CV_32FC3, 1 / 255.0);
            return torch::tensor(torch::ArrayRef<float>(floatArr.ptr<float>(),
                                     floatArr.rows * floatArr.cols * floatArr.channels()),
                c10::TensorOptions().device(device))
                .reshape({ 1, arr.rows, arr.cols, 3 })
                .permute({ 0, 3, 1, 2 })
                .contiguous();
        }
        case CV_32FC1: {
            return torch::tensor(
                torch::ArrayRef<float>(arr.ptr<float>(), arr.rows * arr.cols * arr.channels()),
                c10::TensorOptions().device(device))
                .reshape({ arr.rows, arr.cols })
                .contiguous();
        }
        case CV_32FC3: {
            return torch::tensor(
                torch::ArrayRef<float>(arr.ptr<float>(), arr.rows * arr.cols * arr.channels()),
                c10::TensorOptions().device(device))
                .reshape({ 1, arr.rows, arr.cols, 3 })
                .permute({ 0, 3, 1, 2 })
                .contiguous();
        }
        default: {
            std::ostringstream sout;
            sout << "toTensor(): unrecognized cv::Mat::type() " << arr.type();
            throw std::runtime_error(sout.str());
        }
        }
    }

    cv::Mat toMat(const torch::Tensor &input)
    {

        TORCH_CHECK(input.scalar_type() == torch::ScalarType::Float,
            "toMat(): expected tensor.scalar_type() == torch::ScalarType::Float but got ",
            input.scalar_type());

        auto img = input.squeeze();
        int h = img.size(-2), w = img.size(-1);
        if (img.dim() == 2) {
            img = img.detach().cpu().contiguous();
            cv::Mat res(h, w, CV_32FC1);
            std::memcpy((void *)res.data, img.data_ptr(), sizeof(float) * img.numel());
            return res;
        } else if (img.dim() == 3) {
            img = img.detach().permute({ 1, 2, 0 }).cpu().contiguous();
            cv::Mat res(h, w, CV_32FC3);
            std::memcpy((void *)res.data, img.data_ptr(), sizeof(float) * img.numel());
            return res;
        } else {
            TORCH_CHECK(false, "toMat(): unexpected tensor.dim() == ", img.dim());
        }
        return cv::Mat();
    }

    template <typename S>
    Eigen::Matrix<S, -1, -1> toEigen(const torch::Tensor &input)
    {
        TORCH_CHECK(input.dim() == 2, "input.dim() should be 2, but is ", input.dim());
        auto inp = input.detach().cpu();
        S *data = inp.contiguous().data_ptr<S>();
        return Eigen::Map<Eigen::Matrix<S, -1, -1, Eigen::RowMajor>>(
            data, inp.size(0), inp.size(1));
    }

    template <typename S>
    torch::Tensor toTensor(const Eigen::Matrix<S, -1, -1> &input, bool cuda = false)
    {
        torch::Device device(cuda && torch::cuda::is_available() ? torch::kCUDA : torch::kCPU);
        return torch::tensor(
            torch::ArrayRef<S>(input.data(), input.size()), c10::TensorOptions().device(device))
            .reshape({ input.cols(), input.rows() })
            .permute({ 1, 0 }) // Col major of Eigen
            .contiguous();
    }

    cv::Mat getRoi(const cv::Mat &src, const cv::Rect &rect)
    {
        int l = std::max(rect.x, 1);
        int t = std::max(rect.y, 1);
        int r = std::min(rect.x + rect.width, src.cols - 1);
        int b = std::min(rect.y + rect.height, src.rows - 1);
        if (r - l <= 0 || b - t <= 0) {
            return cv::Mat();
        }
        auto nrect = cv::Rect(l, t, r - l, b - t);
        return src(nrect);
    }

} // namespace utils
