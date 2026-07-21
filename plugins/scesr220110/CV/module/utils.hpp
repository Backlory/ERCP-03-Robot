#pragma once

#include <cmath>
#include <chrono>
#include <ciso646>
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>

namespace F = torch::nn::functional;

namespace module {

    namespace utils {

        // NOTE: color are of GB mode.
        const int __CITY_PALETTE__[][3] = {
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14},
            {44, 160, 44},
            {214, 39, 40},
            {148, 103, 189},
            {140, 86, 75},
            {227, 119, 194},
            {127, 127, 127},
            {188, 189, 34},
            {23, 190, 207},
            {31, 119, 180},
            {255, 127, 14}};

        typedef struct BoxInfo {
            float x1;
            float y1;
            float x2;
            float y2;
            float score;
            int label;
        } BoxInfo;

        static double time() {
            namespace sc = std::chrono;
            sc::system_clock::duration d = sc::system_clock::now().time_since_epoch();
            sc::seconds s = sc::duration_cast<sc::seconds>(d);
            double tv_sec = s.count();
            double tv_usec = sc::duration_cast<sc::microseconds>(d - s).count();
            return tv_sec + tv_usec * 1e-6;
        }

        static torch::Tensor to_tensor(const cv::Mat &arr, bool batched = true) {
            switch (arr.type()) {
                case CV_8UC3: {
                    cv::Mat floatArr;
                    arr.convertTo(floatArr, CV_32FC3, 1 / 255.0);
                    auto tensor = torch::tensor(
                                      torch::ArrayRef<float>(
                                          floatArr.ptr<float>(),
                                          floatArr.rows * floatArr.cols * floatArr.channels()))
                                      .reshape(torch::IntArrayRef{arr.rows, arr.cols, 3})
                                      .permute({2, 0, 1})
                                      .contiguous();
                    if (batched) tensor.unsqueeze_(0);
                    return std::move(tensor);
                }
                case CV_32FC1: {
                    auto tensor = torch::tensor(
                                      torch::ArrayRef<float>(
                                          arr.ptr<float>(),
                                          arr.rows * arr.cols * arr.channels()))
                                      .reshape(torch::IntArrayRef{arr.rows, arr.cols})
                                      .contiguous();
                    if (batched) tensor.unsqueeze_(0);
                    return std::move(tensor);
                }
                case CV_32FC3: {
                    auto tensor = torch::tensor(
                                      torch::ArrayRef<float>(
                                          arr.ptr<float>(),
                                          arr.rows * arr.cols * arr.channels()))
                                      .reshape(torch::IntArrayRef{arr.rows, arr.cols, 3})
                                      .permute({2, 0, 1})
                                      .contiguous();
                    if (batched) tensor.unsqueeze_(0);
                    return std::move(tensor);
                }
                default: {
                    std::ostringstream sout;
                    sout << "toTensor(): unrecognized cv::Mat::type() " << arr.type();
                    throw std::runtime_error(sout.str());
                }
            }
        }

        static cv::Mat to_mat(const torch::Tensor &input) {

            TORCH_CHECK(input.scalar_type() == torch::ScalarType::Float or
                            input.scalar_type() == torch::ScalarType::Int or
                            input.scalar_type() == torch::ScalarType::Long,
                        "toMat(): expected tensor.scalar_type() is Float, Int or Long but got ", input.scalar_type());

            auto img = input.squeeze();
            int h = img.size(-2), w = img.size(-1);
            if (input.scalar_type() == torch::ScalarType::Float) {
                if (img.dim() == 2) {
                    img = img.detach().cpu().contiguous();
                    cv::Mat res(h, w, CV_32FC1);
                    std::memcpy((void *)res.data, img.data_ptr(), sizeof(float) * img.numel());
                    return res;
                } else if (img.dim() == 3) {
                    img = img.detach().permute({1, 2, 0}).cpu().contiguous();
                    cv::Mat res(h, w, CV_32FC3);
                    std::memcpy((void *)res.data, img.data_ptr(), sizeof(float) * img.numel());
                    return res;
                }
            } else if (input.scalar_type() == torch::ScalarType::Int or input.scalar_type() == torch::ScalarType::Long) {

                if (img.max().item<int>() > 255) {
                    TORCH_WARN_ONCE("to_mat() will convert type ", input.scalar_type(), " to lower uint8 for cv::Mat.");
                }

                img = img.to(torch::kUInt8);
                if (img.dim() == 2) {
                    img = img.detach().cpu().contiguous();
                    cv::Mat res(h, w, CV_8UC1);
                    std::memcpy((void *)res.data, img.data_ptr(), sizeof(uint8_t) * img.numel());
                    return res;
                } else if (img.dim() == 3) {
                    img = img.detach().permute({1, 2, 0}).cpu().contiguous();
                    cv::Mat res(h, w, CV_8UC3);
                    std::memcpy((void *)res.data, img.data_ptr(), sizeof(uint8_t) * img.numel());
                    return res;
                }
            } else {
                TORCH_CHECK(false, "toMat(): unexpected tensor.dim() == ", img.dim());
            }
            return cv::Mat();
        }

        static cv::Mat get_roi(const cv::Mat &src, const cv::Rect &rect) {
            int l = std::max(rect.x, 1);
            int t = std::max(rect.y, 1);
            int r = std::min(rect.x + rect.width, src.cols - 1);
            int b = std::min(rect.y + rect.height, src.rows - 1);
            if (r - l <= 0 || b - t <= 0) { return cv::Mat(); }
            auto nrect = cv::Rect(l, t, r - l, b - t);
            return src(nrect);
        }

        static std::vector<BoxInfo> tensor_to_boxinfo(const torch::Tensor &tensor, bool remove_redundant = true) {

            std::vector<BoxInfo> boxes;
            if (!(tensor.dim() == 2)) return boxes;
            if (!(tensor.size(1) == 6)) return boxes;

            std::map<int, std::shared_ptr<BoxInfo>> bins;

            auto pred = tensor.cpu().to(at::kFloat);
            for (int i = 0; i < tensor.size(0); ++i) {
                BoxInfo box;
                box.x1 = pred.index({i, 0}).item<float>();
                box.y1 = pred.index({i, 1}).item<float>();
                box.x2 = pred.index({i, 2}).item<float>();
                box.y2 = pred.index({i, 3}).item<float>();
                box.score = pred.index({i, 4}).item<float>();
                box.label = int(pred.index({i, 5}).item<float>());
                if (remove_redundant) {
                    // Remain best scored box
                    if (bins.find(box.label) == bins.end() || box.score > bins[box.label]->score) {
                        bins[box.label] = std::make_shared<BoxInfo>(box);
                    }
                } else {
                    boxes.push_back(std::move(box));
                }
            }
            // Move from map to boxes
            if (remove_redundant) {
                for (auto item : bins) {
                    boxes.push_back(*item.second);
                }
            }
            return boxes;
        }

        static void draw_bbox(cv::Mat &image, const cv::Rect &bbox, const std::string text, const cv::Scalar &color = cv::Scalar(0, 0, 255)) {

            cv::rectangle(image, bbox, color, 2);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 2, &baseLine);

            // Limit label inner image
            cv::Size txtsize(label_size.width, label_size.height + baseLine);
            int x = bbox.x, y = bbox.y - txtsize.height;
            x = std::max(x, 0);
            x = std::min(x, image.cols - txtsize.width);
            y = std::max(y, 0);
            y = std::min(y, image.rows - txtsize.height);

            cv::rectangle(image, cv::Rect(cv::Point(x, y), txtsize), color, -1);
            cv::putText(image, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
        }

        static cv::Mat draw_bboxes(
            const cv::Mat &bgr,
            const std::vector<BoxInfo> &bboxes,
            const std::vector<std::string> class_names,
            const cv::Rect &effect_roi = {0, 0, 0, 0},
            const std::vector<std::string> exclude = std::vector<std::string>(),
            cv::Point offset = {0, 0},
            int color_offset = 0) {

            cv::Mat image = bgr.clone();

            int src_w = image.cols;
            int src_h = image.rows;
            int dst_w = effect_roi.width > 0 ? effect_roi.width : bgr.cols;
            int dst_h = effect_roi.height > 0 ? effect_roi.height : bgr.rows;
            float width_ratio = (float)src_w / (float)dst_w;
            float height_ratio = (float)src_h / (float)dst_h;

            for (size_t i = 0; i < bboxes.size(); i++) {
                const BoxInfo &info = bboxes[i];
                assert(info.label < class_names.size());
                std::string name = class_names[info.label];

                if (std::find(exclude.begin(), exclude.end(), name) != exclude.end()) {
                    continue;
                }

                cv::Scalar color = cv::Scalar(
                    __CITY_PALETTE__[(uint8_t)info.label + color_offset][2],
                    __CITY_PALETTE__[(uint8_t)info.label + color_offset][1],
                    __CITY_PALETTE__[(uint8_t)info.label + color_offset][0]);

                cv::Rect bbox(cv::Point((info.x1 - effect_roi.x) * width_ratio, (info.y1 - effect_roi.y) * height_ratio),
                              cv::Point((info.x2 - effect_roi.x) * width_ratio, (info.y2 - effect_roi.y) * height_ratio));

                auto text = fmt::format("{}-{:.02f}", name, info.score * 100);
                draw_bbox(image, bbox, text, color);
            }
            return image;
        }

    }  // namespace utils
}  // namespace module
