//#pragma once
//#include <torch/torch.h>
//#include <torchvision/ops/ops.h>
//#include <opencv2/opencv.hpp>
//#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
//#include <filesystem>
//#include "utils.hpp"
//
//using namespace torch::indexing;
//namespace fs = std::filesystem;
//
//namespace module {
//
//    class Detector {
//    public:
//        Detector(const torch::DeviceType &device = torch::kCUDA)
//            : device_type(device) {}
//        Detector(const Detector &) = delete;
//        ~Detector()                = default;
//
//    public:
//        virtual bool load(const std::string &path) {
//            try {
//                // Deserialize the ScriptModule from a file using torch::jit::load().
//                torch::NoGradGuard no_grad;
//                auto               model_path = fs::path(path);
//                if (fs::is_regular_file(model_path)) {
//                    module = torch::jit::load(model_path.string(), device_type);
//                    module.eval();
//                    loaded = true;
//                }
//            } catch (std::exception &e) { std::cout << e.what() << std::endl; }
//            return loaded;
//        }
//
//        virtual void warmup(int iter, bool infer) {
//            auto x = torch::randn({1, 3, input_height_, input_width_}, torch::TensorOptions().dtype(torch::kFloat32).device(device_type));
//            for (int i = 0; i < iter; ++i) {
//                if (infer)
//                    inference(x);
//                else
//                    module.forward({torch::TensorList(x)});
//            }
//        }
//
//        virtual std::vector<utils::BoxInfo> inference(const cv::Mat &src) = 0;
//
//    private:
//        torch::Tensor preprocess(const cv::Mat &src);
//
//        virtual std::vector<utils::BoxInfo> inference(torch::Tensor input) = 0;
//
//    private:
//        ///< Input tensor size (square) of the net.
//        const int input_width_  = 320;
//        const int input_height_ = 320;
//
//    private:
//        torch::jit::script::Module module;
//        torch::DeviceType          device_type;
//        bool                       loaded = false;
//    };
//
//}  // namespace module
