#pragma once
#include <string>
#include <vector>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include "utils.hpp"

using namespace module;

namespace nanodet {

    class Nanodet {
    public:
        static const std::vector<std::string> classes;

    public:
        Nanodet(const torch::DeviceType &device = torch::kCUDA)
            : device_type(device),
              mean_(torch::tensor(torch::ArrayRef<float>(cmean_, 3), torch::TensorOptions().dtype(torch::kFloat32).device(device)).view({1, 3, 1, 1})),
              std_(torch::tensor(torch::ArrayRef<float>(cstd_, 3), torch::TensorOptions().dtype(torch::kFloat32).device(device)).view({1, 3, 1, 1})) {}

        Nanodet(const Nanodet &) = delete;
        ~Nanodet() = default;

    public:
        bool load(const std::string &path);

        bool is_loaded() {
            return loaded;
        }

        void warmup(int iter = 3, bool infer = false);

        /// <summary>
        /// Inference objects in image.
        /// </summary>
        /// <param name="src">Input image.</param>
        /// <param name="score_thre">Output threshold for object score.
        ///     Higher than this threshold will be outputed.</param>
        /// <param name="iou_thre">NMS IoU threshold for object score.</param>
        /// <param name="score_pre">NMS threshold for object score.
        ///     Higher than this threshold will be preserved to be selecting.</param>
        /// <returns></returns>
        std::vector<utils::BoxInfo> inference(const cv::Mat &src, float score_thre = 0.5, float iou_thre = 0.5, float score_pre = 0.25);

    private:
        torch::Tensor preprocess(const cv::Mat &src);

        std::vector<utils::BoxInfo> inference(torch::Tensor input, float score_thre, float iou_thre, float score_pre);

        void decode_bbox(
            const std::vector<torch::Tensor> &cls_preds,
            const std::vector<torch::Tensor> &bbox_preds,
            std::vector<std::vector<utils::BoxInfo>> &results,
            float score_thre,
            float iou_threshold,
            int max_num = -1,
            int max_num_nms = 1000);

    private:
        ///< Strides for each output channel.
        const std::vector<int> strides_{8, 16, 32};
        ///< Input tensor size (square) of the net.
        const int input_size_ = 320;
        ///< Max index of regressive value.
        const int reg_max_ = 7;
        ///< Normalization param for input image.
        const float cmean_[3] = {0.406, 0.456, 0.485};
        const float cstd_[3] = {0.225, 0.224, 0.229};
        const torch::Tensor mean_;
        const torch::Tensor std_;
        //const auto mean_ = torch::Tensor({103.53, 116.28, 123.675});
        //const auto std_  = torch::Tensor({57.375, 57.12, 58.395});

    private:
        torch::jit::script::Module module;
        torch::DeviceType device_type;
        bool loaded = false;
    };

}  // namespace nanodet
