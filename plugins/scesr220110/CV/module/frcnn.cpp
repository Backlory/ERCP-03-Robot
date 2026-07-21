#pragma once

#include <string>
#include <memory>
#include <vector>
#include <cmath>
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <filesystem>
#include <torch/torch.h>
#include <torch/script.h>
#include <torchvision/vision.h>
#include <torchvision/ops/roi_pool.h>
#include "frcnn.hpp"

namespace fs = std::filesystem;
using namespace module;

namespace frcnn {

    const std::vector<std::string> frcnn::classes = {
        "__background__",
        "Great curvature",
        "Pylorus",
        "Larynx",
        "Vocal cords",
        "Fundus of stomach",
        "Endoscope",
        "Duodenum",
        "Esophagus"};

    bool frcnn::load(const std::string &path) {

        try {
            // Deserialize the ScriptModule from a file using torch::jit::load().
            torch::NoGradGuard no_grad;
            auto               model_path = fs::path(path);
            if (fs::is_regular_file(model_path)) {
                module = torch::jit::load(model_path.string());
                module.eval();
                loaded = true;
            }
        } catch (std::exception &e) { std::cout << e.what() << std::endl; }
        return loaded;
    }

    void frcnn::warmup(int iter, bool infer) {
        if (infer) {
            auto x = cv::Mat(input_size_, input_size_, CV_8UC3);
            for (int i = 0; i < iter; ++i) {
                inference(x);
            }
        } else {
            auto x = torch::randn({3, input_size_, input_size_}, torch::TensorOptions().dtype(torch::kFloat32).device(device_type));
            for (int i = 0; i < iter; ++i) {
                module.forward({torch::TensorList(x)});
            }
        }
    }

    std::vector<utils::BoxInfo> frcnn::inference(const cv::Mat &src, float score_thre) {
        // Input: list of images
        // Output: Tuple[losses, list[BoxList]];
        //         BoxList contains `boxes`, `scores` and `labels`

        assert(loaded);
        // !!!!!!!!
        cv::Mat input;
        cv::cvtColor(src, input, cv::COLOR_BGR2RGB);

        // ------- DETECTION ------- //
        // Convert input to tensor
        auto images = utils::to_tensor(input, false).to(device_type);

        // Predict with model
        std::vector<torch::Tensor> _imgs{images};
        auto                       _tup = module.forward({torch::TensorList(_imgs)}).toTuple()->elements();

        // Fetch detection target
        auto boxlist = _tup[1].toListRef()[0].toGenericDict();
        auto boxes   = boxlist.at("boxes").toTensor().contiguous().cpu();
        auto labels  = boxlist.at("labels").toTensor().cpu();
        auto scores  = boxlist.at("scores").toTensor().cpu();

        auto   ret = std::vector<utils::BoxInfo>();
        size_t sz  = boxes.size(0);

        for (int i = 0; i < sz; ++i) {
            float score = scores[i].item().toFloat();
            if (score > score_thre) {
                auto bb  = boxes[i].to(torch::kFloat32).data_ptr<float>();
                int  lab = labels[i].item().toInt();

                ret.emplace_back(utils::BoxInfo{bb[0], bb[1], bb[2], bb[3], score, lab});
            }
        }

        return std::move(ret);
    }
}  // namespace frcnn
