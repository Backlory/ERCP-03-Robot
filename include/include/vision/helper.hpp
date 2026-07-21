#pragma once
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>
#ifdef USING_OPENCV
#include <opencv2/opencv.hpp>
#endif
#ifdef USING_TORCH
#include <torch/torch.h>
#endif

namespace helper {

#ifdef USING_OPENCV
    cv::Mat LoadIntrinsics()
    {
        auto node = YAML::LoadFile("data/olympus.yaml");
        cv::Mat intrinsics = cv::Mat(4, 4, CV_32FC1);
        for (int i = 0; i < node["intrinsics"].size(); ++i) {
            auto in = node["intrinsics"][i].as<std::vector<double>>();
            for (int j = 0; j < in.size(); ++i) {
                intrinsics.at<float>(i, j) = in[j];
            }
        }
        return intrinsics;
    }

    cv::Mat LoadMask(bool invert = false, int size = -1)
    {
        cv::Mat mask_src = cv::imread("data/endomask.png", 0);
        cv::Mat mask;
        if (size > 0 && mask.rows != size) {
            cv::resize(mask_src, mask, { size, size });
        }
        return mask;
    }
#endif

#ifdef USING_TORCH
    torch::Tensor load(std::string filename)
    {
        std::ifstream input(filename, std::ios::binary);
        std::vector<char> bytes(
            (std::istreambuf_iterator<char>(input)), (std::istreambuf_iterator<char>()));
        input.close();
        return torch::pickle_load(bytes).toTensor();
    }

    void save(torch::Tensor x, std::string filename)
    {
        std::ofstream output(filename, std::ios::out | std::ios::binary);
        std::vector<char> bytes = torch::pickle_save(x);
        std::copy(bytes.cbegin(), bytes.cend(), std::ostream_iterator<char>(output));
        output.close();
    }

    void save_ply(std::string filename, torch::Tensor x)
    {
        std::ofstream ofs(filename);
        ofs << fmt::format("ply\n"
                           "format ascii 1.0\n"
                           "element vertex {}\n"
                           "property double x\n"
                           "property double y\n"
                           "property double z\n",
            x.size(-1));
        if (x.size(-2) >= 6) {
            ofs << "property uchar red\n"
                   "property uchar green\n"
                   "property uchar blue\n";
        }
        ofs << "end_header\n";
        for (int i = 0; i < x.size(-1); ++i) {
            // Eigen::Matrix<float, 6, 1> vec = pcl.col(i);
            float x1 = x.index({ 0, i }).item().toFloat();
            float x2 = x.index({ 1, i }).item().toFloat();
            float x3 = x.index({ 2, i }).item().toFloat();
            ofs << fmt::format("{:.06f} {:.06f} {:.06f}\n", x1, x2, x3);
            if (x.size(-2) >= 6) {
                int c1 = x.index({ 3, i }).item().toFloat() * 255;
                int c2 = x.index({ 4, i }).item().toFloat() * 255;
                int c3 = x.index({ 5, i }).item().toFloat() * 255;
                ofs << fmt::format("{} {} {}\n", c1, c2, c3);
            }
        }
    }

    torch::Tensor load_intrinsics(const std::string &file)
    {
        auto node = YAML::LoadFile(file);
        auto intrinsics = torch::ones({ 3, 3 }, c10::TensorOptions().dtype(c10::kFloat));
        for (int i = 0; i < node["intrinsics"].size() && i < 3; ++i) {
            auto in = node["intrinsics"][i].as<std::vector<float>>();
            intrinsics.index_put_({ i, 0 }, in[0]);
            intrinsics.index_put_({ i, 1 }, in[1]);
            intrinsics.index_put_({ i, 2 }, in[2]);
        }
        return intrinsics;
    }

    torch::Tensor backproj_grid(
        torch::Tensor iK, int width, int height = -1, c10::DeviceType dt = c10::kCPU)
    {
        torch::NoGradGuard();
        if (iK.dim() != 2 || iK.size(-2) != 3 || iK.size(-1) != 3) {
            throw std::runtime_error("Suppose `iK` to be a 3x3 matrix.");
        }
        height = height <= 0 ? width : height;
        auto opt = c10::TensorOptions().dtype(c10::kFloat).device(dt);
        auto x = torch::linspace(0, 1.0, width, opt);
        auto y = torch::linspace(0, 1.0, height, opt);
        auto yx = torch::meshgrid({ x, y });
        auto xyz = torch::stack({ yx[1], yx[0], torch::ones_like(yx[0]) }, -3);
        xyz = iK.matmul(xyz.view({ 3, -1 }));
        return F::normalize(xyz.reshape({ 3, height, width }), F::NormalizeFuncOptions().dim(-3));
    }
#endif

} // namespace helper
