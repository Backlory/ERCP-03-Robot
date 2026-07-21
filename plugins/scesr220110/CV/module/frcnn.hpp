#pragma once
#include <string>
#include <vector>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include "utils.hpp"

using namespace module;

namespace frcnn {

    class frcnn {
    public:
        static const std::vector<std::string> classes;

    public:
        frcnn(const torch::DeviceType &device = torch::kCUDA)
            : device_type(device) {}
        frcnn(const frcnn &) = delete;
        ~frcnn() {}

    public:
        bool load(const std::string &path);

        void warmup(int iter = 3, bool infer = true);

        std::vector<utils::BoxInfo> inference(const cv::Mat &src, float score_thre = 0.5);

    private:
        torch::jit::script::Module module;
        torch::DeviceType          device_type;
        bool                       loaded = false;

        const int input_size_ = 320;
    };

}  // namespace frcnn
