#pragma once
#include <string>
#include <vector>
#include <torch/torch.h>
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <filesystem>
#include "utils.hpp"

namespace fs = std::filesystem;
using namespace module;

namespace elm {

    template <int D>
    class ELM {
    public:
        ELM(const torch::DeviceType &device = torch::kCPU)
            : device_type(device) {}
        ELM(const ELM &) = delete;
        ~ELM() {}

    public:
        bool ELM::load(const std::string &path) {

            try {
                // Deserialize the ScriptModule from a file using torch::jit::load().
                torch::NoGradGuard no_grad;
                auto model_path = fs::path(path);
                if (fs::is_regular_file(model_path)) {
                    module = torch::jit::load(model_path.string());
                    module.eval();
                    loaded = true;
                }
            } catch (std::exception &e) { std::cout << e.what() << std::endl; }
            return loaded;
        }

        int ELM::inference(const std::array<float, D> &src) {

            auto tensor = torch::tensor(
                              torch::ArrayRef<float>(src.data(), D))
                              .reshape({1, D})
                              .contiguous();
            // Predict with model
            auto res = module.forward({tensor}).toTensor();
            // Fetch detection target
            return res.view(-1).argmax().item().toInt();
        }

    private:
        torch::jit::script::Module module;
        torch::DeviceType device_type;
        bool loaded = false;
    };

}  // namespace elm
