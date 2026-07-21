#include "cvtask.hpp"
#include "module/nanodet.hpp"
#include "module/elml.hpp"
#include "module/elm.hpp"
#include "module/utils.hpp"
#include "utils.h"
#include "cvui.hpp"
#include "../Robot/VBot.hpp"
#include "recog.hpp"

using namespace ilsr;
#define DEBUG

namespace recog {

    // 归一化：ud, lr, rot, torq, length, q2, q3, q4
    // const std::array<double, 8> N_OCSVM = {{180, 90, 200, 10, 1000, 131072, 131072, 131072}};
    const std::array<double, 8> N_OCSVM = { {180, 90, 200, 30, 1000, 360, 360, 360} };

    inline static double score_func(int index) {
        return std::exp(1.0 * index);
    }

    inline static double simple_relu(double value) {
        return std::clamp<double>(value, 0.0, 1.0);
    }

    std::vector<operation_t> maping_opeartion(std::string name) {
        std::vector<operation_t> results;
        for (const auto &act : ActionMapper) {
            if (act.second.features.find(name) != act.second.features.end()) {
                results.emplace_back(operation_t{act.second.abbr, act.second.features.at(name), act.second.min_length, act.second.max_length});
            }
        }
        std::sort(results.begin(), results.end(), [](const operation_t &a, const operation_t &b) {
            return a.voting > b.voting;
        });
        return results;
    }

    CvDetector::CvDetector() {

        bool ret = false;
        // 修改常量数值
        auto score = const_cast<double *>(&MaxHistoryScore);
        *score = 0.0;
        for (int i = 1; i <= MaxHistorySize; ++i) {
            *score += score_func(-i);
        }

        torch::NoGradGuard no_grad;
        try {
#ifdef DEBUG
            std::cout << "Loading for FasterRCNN ..." << std::endl;
#endif  // DEBUG \
            // ret = m_detector.load("data/faster-rcnn_0702-ep0500-vgg16-im1049aohua-plus.pt");
            ret = m_detector.load("data/scesr/nanodet-t_320x320_cuda_220111-2.pth");
#ifdef DEBUG
            assert(ret);
#endif  // DEBUG
            std::cout << "Warming up detector ..." << std::endl;
            m_detector.warmup(5, true);
            std::cout << "Warm up finished." << std::endl;
        } catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
            exit(-1);
        }

        // ELM
#ifdef DEBUG
        std::cout << "Loading for ELM ..." << std::endl;
#endif  // DEBUG                              \
    //read_binary("data/scesr/in_weight.w", m_inw); \
    //read_binary("data/scesr/bais.w", m_bais);     \
    //read_binary("data/scesr/out_weight.w", m_ouw);
        ret = m_classifier.load("data/scesr/elm_220119.pth");
#ifdef DEBUG
        assert(ret);
#endif  // DEBUG

        // OC-SVM
#ifdef DEBUG
        std::cout << "Loading for SVM ..." << std::endl;
#endif  // DEBUG
        ret = m_discriminator.load("data/scesr/ocsvm_220118.model");
#ifdef DEBUG
        assert(ret);
#endif  // DEBUG
    }

    void CvDetector::process(cv::Mat &frame) {

        // Get status
        auto bot = VirtualRobot::get_instance();
        const double ud = bot ? bot->get_bend_ud() : 0;
        const double lr = bot ? bot->get_bend_lr() : 0;
        const double rot = bot ? bot->get_rotation() : 0;
        const double torq = bot ? bot->get_torque() : 0;
        const double leng = bot ? bot->get_length() : 0;
        const double q2 = bot ? bot->get_q2() : 0;
        const double q3 = bot ? bot->get_q3() : 0;
        const double q4 = bot ? bot->get_q4() : 0;

        cvf_label res;
        bool valid = false;

        // A.检测器: 图像中检测目标
        cv::Mat input;
        cv::resize(frame, input, cv::Size(320, 320));

        std::vector<utils::BoxInfo> boxes;
        try {
            boxes = m_detector.inference(input, 0.5, 0.45, 0.25);
        } catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
            return;
        }

        res.extra.targets.insert(res.extra.targets.end(), boxes.begin(), boxes.end());
        if (boxes.size() > 0) {
            res.extra.curr_label = boxes[0].label;
            res.extra.curr_score = boxes[0].score;
            valid = true;
        }

        // C.SVM: 判别操作合法性
        std::array<double, 8> svmd = {ud, lr, rot, torq, leng, q2, q3, q4};
        // 标准化
        for (int i = 0; i < svmd.size(); ++i) {
            svmd[i] /= N_OCSVM[i];
        }
        res.ope_valid_score = m_discriminator.inference(svmd);
        
        for (int i = 0; i < 8; ++i) {
            std::cout << fmt::format("{:.03f}, ", svmd[i]);
        }
        
        std::cout << fmt::format("{:.03f}, ", res.ope_valid_score);

        // B.ELM超限学习机: 判别操作内容
        std::array<float, 9> data = { ud, lr, rot, torq, leng, res.extra.curr_label, q2, q3, q4 };
        res.ope_case = (operation_tag_t)(m_classifier.inference(data) + 1);

        std::cout << fmt::format("{:d}, ", (int)res.ope_case);
        std::cout << std::endl;


        // Save results
        {
            // Writer lock
            boost::upgrade_lock<boost::shared_mutex> lock(m_access);
            boost::upgrade_to_unique_lock<boost::shared_mutex> unique(lock);
            m_result = res;
            m_ftime = ilsr::Time::wall_time();
        }

#ifdef DEBUG
        // std::cout << result;
        // std::cout << cv::countNonZero(result) * 1.0 / result.rows;
        // double dt = (1.0 * end - start) / CLOCKS_PER_SEC;
        // std::cout << "; dt = " << dt << std::endl;
#endif
    }

    cvf_label CvDetector::get_labels(double overtime) {
        // Reader lock
        boost::shared_lock<boost::shared_mutex> lock(m_access);
        if (ilsr::Time::wall_time() - m_ftime < overtime) { return m_result; }
        return cvf_label();
    }

    //operation_t CvDetector::get_operation(double overtime) {
    //    // Reader lock
    //    boost::shared_lock<boost::shared_mutex> lock(m_access);
    //    if (ilsr::GetTime::GetLocalTime() - m_ftime < overtime) { return m_result.ope_case; }
    //    return operation_t();
    //}

    CvDetector::~CvDetector() {
        close();
    }

    void CvDetector::on_update(cv::Mat &) {}

#define MUST   0.9
#define ALMOST 0.75
#define POSSI  0.5
#define NEARLY 0.25
#define MAYBE  0.1

    // Note: Y axis of center increase inversely with real angle.
    const std::map<operation_tag_t, defined_operation_t> ActionMapper = {
        {TM, defined_operation_t({TM,
                                  "Throughing Mouth",
                                  150,
                                  400,
                                  std::map<std::string, double>{
                                      {"Larynx", MUST},
                                      {"Vocal cords", MUST},
                                      {"Tongue", MUST},
                                      {"Esophagus", NEARLY}},
                                  operation_workspace_t{
                                      Ellipse,
                                      cv::Point(0, 50),
                                      cv::Size(160, 140),
                                  }})},
        {FE, defined_operation_t({FE,
                                  "Forwarding Esophagus",
                                  200.0,
                                  600.0,
                                  std::map<std::string, double>{
                                      {"Esophagus", ALMOST}},
                                  operation_workspace_t{
                                      Ellipse,
                                      cv::Point(0, 0),
                                      cv::Size(100, 100),
                                  }})},
        {TC, defined_operation_t({TC,
                                  "Throughing Cardia",
                                  400,
                                  600,
                                  std::map<std::string, double>{
                                      {"Esophagus", MAYBE},
                                      {"Great curvature", NEARLY}},
                                  operation_workspace_t{
                                      Ellipse,
                                      cv::Point(0, 0),
                                      cv::Size(120, 120),
                                  }})},
        {OG, defined_operation_t({OG,
                                  "Observing Gastric_body",
                                  500,
                                  800,
                                  std::map<std::string, double>{
                                      {"Great curvature", POSSI},
                                      {"Gastric angle", POSSI}},
                                  operation_workspace_t{
                                      All}})},
        {OA, defined_operation_t({OA,
                                  "Observing Antrum",
                                  600,
                                  950,
                                  std::map<std::string, double>{
                                      {"Pyloric antrum", POSSI},
                                      {"Pylorus", POSSI}},
                                  operation_workspace_t{
                                      All}})},
        {FD, defined_operation_t({FD,
                                  "Forwarding Duodenum",
                                  700,
                                  1000,
                                  std::map<std::string, double>{
                                      {"Duodenum", MUST}},
                                  operation_workspace_t{
                                      Ellipse,
                                      cv::Point(30, 90),
                                      cv::Size(180, 180),
                                  }})},
        {OF, defined_operation_t({OF,
                                  "Observing Fundus",
                                  550,
                                  850,
                                  std::map<std::string, double>{
                                      {"Fundus of stomach", MUST},
                                      {"Endoscope", MUST}},
                                  operation_workspace_t{
                                      All}})},
        {Undefined, defined_operation_t({Undefined,
                                         "Unknown",
                                         -1,
                                         -1,
                                         std::map<std::string, double>(),
                                         operation_workspace_t()})}};
}  // namespace recog
