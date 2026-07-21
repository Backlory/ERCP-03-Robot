#pragma once
#include <torch/torch.h>
#include <torch/script.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

namespace vision {

    namespace fs = boost::filesystem;
    namespace F = torch::nn::functional;
    namespace jit = torch::jit::script;
    using tensor = torch::Tensor;

    class single_network {
    public:
        single_network(const std::string &name, const torch::DeviceType &device, int verbose = 1)
            : dump_name(name)
            , device_type(device)
            , verbose(verbose)
        {
        }
        single_network(const single_network &) = delete;
        ~single_network() {}

    public:
        bool load(const std::string &path)
        {
            try {
                auto p = fs::absolute(path);
                if (fs::is_directory(p)) {
                    p = p / dump_name;
                } else {
                    auto ext = p.extension().string();
                    boost::algorithm::to_lower(ext);
                    if (ext != ".pth" && ext != ".pt") {
                        throw std::runtime_error(
                            ("File has wrong extension " + ext + ", need .pth or .pt.").c_str());
                    }
                }
                if (!fs::exists(p)) {
                    throw std::runtime_error("Network weight file " + p.string() + " not exists.");
                }

                // Deserialize the ScriptModule from a file using torch::jit::load().
                if (verbose > 1)
                    std::cout << "Load weights from " << p.string() << " ..." << std::endl;
                torch::NoGradGuard no_grad;
                auto _module
                    = std::make_shared<jit::Module>(std::move(torch::jit::load(p.string())));
                _module->to(device_type);
                _module->eval();
                std::lock_guard<decltype(m_mutex)> _(m_mutex);
                module.swap(_module);
                return true;

            } catch (std::exception &e) {
                if (verbose > 0)
                    std::cerr << e.what() << std::endl;
                std::lock_guard<decltype(m_mutex)> _(m_mutex);
                module = nullptr;
            }
            return false;
        }

        bool unload()
        {
            try {
                std::lock_guard<decltype(m_mutex)> _(m_mutex);
                module.reset();
                return true;
            } catch (std::exception &e) {
                if (verbose > 0)
                    std::cerr << e.what() << std::endl;
            }
            return false;
        }

        bool is_loaded() const
        {
            std::lock_guard<decltype(m_mutex)> _(m_mutex);
            return module != nullptr;
        }

        virtual void warm_up(int times = 3) const = 0;

        virtual tensor forward(const cv::Mat &src) = 0;

    protected:
        const std::string dump_name;
        const torch::DeviceType device_type;
        const int verbose;
        std::shared_ptr<jit::Module> module;
        mutable std::mutex m_mutex;
    };

} // namespace vision
