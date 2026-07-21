#include <iostream>
#include <algorithm>
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <filesystem>
#include <torch/script.h>
#include <torchvision/vision.h>
#include "nms.hpp"
#include "nanodet.hpp"

namespace fs = std::filesystem;
using namespace module;

namespace nanodet {

    const std::vector<std::string> Nanodet::classes = {
        "Endoscope",
        "Tongue",
        "Larynx",
        "Vocal cords",
        "Esophagus",
        "Fundus of stomach",
        "Gastric angle",
        "Great curvature",
        "Pyloric antrum",
        "Pylorus",
        "Duodenum",
        "Lesion"};

    bool Nanodet::load(const std::string &path) {
        try {
            // Deserialize the ScriptModule from a file using torch::jit::load().
            torch::NoGradGuard no_grad;
            auto model_path = fs::path(path);
            if (fs::is_regular_file(model_path)) {
                module = torch::jit::load(model_path.string(), device_type);
                module.eval();
                loaded = true;
            }
        } catch (std::exception &e) { std::cout << e.what() << std::endl; }
        return loaded;
    }

    void Nanodet::warmup(int iter, bool infer) {
        auto x = torch::randn({1, 3, input_size_, input_size_}, torch::TensorOptions().dtype(torch::kFloat32).device(device_type));
        for (int i = 0; i < iter; ++i) {
            if (infer)
                inference(x, 0.5, 0.5, 0.5);
            else
                module.forward({torch::TensorList(x)});
        }
    }

    torch::Tensor Nanodet::preprocess(const cv::Mat &src) {
        auto image = utils::to_tensor(src).to(device_type);
        if (input_size_ != image.size(2) || input_size_ != image.size(3)) {
            image = F::interpolate(image, F::InterpolateFuncOptions()
                                              .size(std::vector<int64_t>{input_size_, input_size_})
                                              .mode(torch::kBilinear)
                                              .align_corners(false));
        }
        image.sub_(mean_);
        image.div_(std_);
        return image;
    }

    std::vector<utils::BoxInfo> Nanodet::inference(const cv::Mat &src, float score_thre, float iou_thre, float score_pre) {
        assert(loaded);
        torch::NoGradGuard no_grad;
        // !!!!!!!! 注意：因为训练的时候Nanodet直接用的cv.imread，所以不需要颜色转换
        // cv::cvtColor(input, input, cv::COLOR_BGR2RGB);
        auto input = preprocess(src);
        return inference(input, score_thre, iou_thre, score_pre);
    }

    std::vector<utils::BoxInfo> Nanodet::inference(torch::Tensor input, float score_thre, float iou_thre, float score_pre) {

        assert(loaded);
        torch::NoGradGuard no_grad;
        auto res = module.forward({input});
        auto outputs = res.toTuple();

        auto cls_preds = outputs->elements()[0].toTensorVector();
        auto box_preds = outputs->elements()[1].toTensorVector();

        std::vector<std::vector<utils::BoxInfo>> results;
        results.resize(classes.size());

        decode_bbox(cls_preds, box_preds, results, score_pre, iou_thre);

        auto dets = std::vector<utils::BoxInfo>();
        for (auto objs : results) {
            if (objs.size() <= 0)
                continue;
            // Remove those not reach score_thre
            objs.erase(std::remove_if(
                           objs.begin(), objs.end(), [score_thre](const utils::BoxInfo &a) {
                               return a.score < score_thre;
                           }),
                       objs.end());
            dets.insert(dets.end(), objs.begin(), objs.end());
        }
        std::sort(dets.begin(), dets.end(), [](const utils::BoxInfo &a, const utils::BoxInfo &b) { return a.score > b.score; });
        return dets;
    }

    /// <summary>
    /// Generate pixel centers of a single stage feature map.
    /// </summary>
    /// <param name="featmap_size">height and width of the feature map</param>
    /// <param name="stride">down sample stride of the feature map</param>
    /// <param name="dtype">data type of the tensors</param>
    /// <param name="device">device of the tensors</param>
    /// <param name="flatten">flatten the x and y tensors</param>
    /// <returns>y and x of the center points</returns>
    torch::Tensor get_single_level_center_point(int h, int w, int stride, const c10::TensorOptions &options, bool flatten = true) {

        auto x_range = (torch::arange(w, options) + 0.5) * stride;
        auto y_range = (torch::arange(h, options) + 0.5) * stride;

        auto yx = torch::meshgrid({y_range, x_range});

        if (flatten) {
            yx[0] = yx.at(0).flatten();
            yx[1] = yx.at(1).flatten();
        }
        return torch::stack({yx[1], yx[0]}, -1);
    }

    torch::Tensor distribution_project(torch::Tensor x, int reg_max = 7) {
        auto project = torch::linspace(0, reg_max, reg_max + 1);
        x = F::softmax(x.reshape({-1, reg_max + 1}), 1);
        x = F::linear(x, project.type_as(x)).reshape({-1, 4});
        return x;
    }

    /// <summary>
    /// Decode distance prediction to bounding box.
    /// </summary>
    /// <param name="">Shape (n, 2), [x, y].</param>
    /// <param name="">Distance from the given point to 4, boundaries (left, top, right, bottom).</param>
    /// <param name="">Shape of the image.</param>
    /// <returns>Tensor: Decoded bboxes.</returns>
    torch::Tensor distance2bbox(torch::Tensor points, torch::Tensor distance, torch::optional<std::tuple<int, int>> max_shape = c10::nullopt) {
        auto x1 = points.select(1, 0) - distance.select(1, 0);
        auto y1 = points.select(1, 1) - distance.select(1, 1);
        auto x2 = points.select(1, 0) + distance.select(1, 2);
        auto y2 = points.select(1, 1) + distance.select(1, 3);
        if (max_shape.has_value()) {
            x1 = x1.clamp(0, std::get<1>(max_shape.value()));
            y1 = y1.clamp(0, std::get<0>(max_shape.value()));
            x2 = x2.clamp(0, std::get<1>(max_shape.value()));
            y2 = y2.clamp(0, std::get<0>(max_shape.value()));
        }
        return torch::stack({x1, y1, x2, y2}, -1);
    }

    void Nanodet::decode_bbox(
        const std::vector<torch::Tensor> &cls_preds,
        const std::vector<torch::Tensor> &bbox_preds,
        std::vector<std::vector<utils::BoxInfo>> &results,
        float score_thre,
        float iou_thre,
        int max_num,
        int max_num_nms) {

        int cls_out_channels = classes.size();
        std::vector<torch::Tensor> mlvl_bboxes;
        std::vector<torch::Tensor> mlvl_scores;

        for (int i = 0; i < 3; ++i) {

            int stride = strides_[i];
            auto cls_pred = cls_preds[i].select(0, 0);
            auto bbox_pred = bbox_preds[i].select(0, 0);

            assert(cls_pred.size(-2) == bbox_pred.size(-2) && cls_pred.size(-1) == bbox_pred.size(-1));

            int feature_h = cls_pred.size(-2);
            int feature_w = cls_pred.size(-1);

            auto center_points = get_single_level_center_point(feature_h, feature_w, stride, cls_pred.options(), true);

            auto scores = cls_pred.permute({1, 2, 0}).reshape({-1, cls_out_channels}).sigmoid();
            bbox_pred = bbox_pred.permute({1, 2, 0});
            bbox_pred = distribution_project(bbox_pred) * stride;

            if (scores.size(0) > max_num_nms) {
                // Get toppest max_num_nms of targets
                auto max_scores = std::get<0>(scores.max(1));
                auto topk_inds = std::get<1>(max_scores.topk(max_num_nms));
                center_points = center_points.index({topk_inds});
                bbox_pred = bbox_pred.index({topk_inds});
                scores = scores.index({topk_inds});
            }

            auto bboxes = distance2bbox(center_points, bbox_pred, std::make_tuple(input_size_, input_size_));

            mlvl_bboxes.emplace_back(std::move(bboxes));
            mlvl_scores.emplace_back(std::move(scores));
        }

        auto mlvl_bboxes_ = torch::cat(mlvl_bboxes);
        auto mlvl_scores_ = torch::cat(mlvl_scores);

        // add a dummy background class at the end of all labels, same with mmdetection2.0
        auto padding = mlvl_scores_.new_zeros({mlvl_scores_.size(0), 1});
        mlvl_scores_ = torch::cat({mlvl_scores_, padding}, 1);

        torch::Tensor dets = multiclass_nms(mlvl_bboxes_, mlvl_scores_, score_thre, iou_thre, max_num);
        for (int i = 0; i < cls_out_channels; ++i) {
            // results
            auto inds = (dets.select(-1, -1).to(torch::kLong) == i);
            auto res = utils::tensor_to_boxinfo(dets.index({inds, Slice(None, 6)}), false);
            results[i].insert(results[i].end(), res.begin(), res.end());
        }
    }
}  // namespace nanodet
