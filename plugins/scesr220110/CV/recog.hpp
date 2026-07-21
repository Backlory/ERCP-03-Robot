#pragma once
#include "cvtask.hpp"
#include "module/nanodet.hpp"
#include "module/elm.hpp"
#include "module/utils.hpp"
#include "module/ocsvm.hpp"

namespace recog {

    typedef enum : int {
        Undefined = 0,
        TM = 1,
        FE,
        TC,
        OG,
        OA,
        FD,
        OF,
        _tag_size
    } operation_tag_t;

    typedef enum : int {
        Disable = 0,
        Rectangle,
        Ellipse,
        All,
        _type_size
    } operation_workspace_type_t;

    typedef struct {
        operation_workspace_type_t type = Disable;  // only for all, rectangle, ellipse
        cv::Point center;
        cv::Point size;
    } operation_workspace_t;

    typedef struct {
        operation_tag_t abbr;
        std::string full;
        double min_length;
        double max_length;
        /// <summary>
        /// Feature nemes and its supporting score that suppoting this operation.
        /// </summary>
        std::map<std::string, double> features;
        /// <summary>
        /// Define workspace constraints for each operation
        /// </summary>
        operation_workspace_t workspace;
    } defined_operation_t;

    /// <summary>
    /// Defined actiuon informations.
    /// </summary>
    extern const std::map<operation_tag_t, defined_operation_t> ActionMapper;

    typedef struct {
        recog::operation_tag_t name = Undefined;
        double voting = -1;
        double min_length = -1;
        double max_length = -1;
        ///< 当前类别
        int label = -1;
        ///< 当前类别分数
        double score = 0;
    } operation_t;

    typedef struct {
        ///< 当前判定动作
        operation_tag_t ope_case = Undefined;
        ///< 当前动作有效性分数
        double ope_valid_score = 0;
        struct {
            ///< 当前识别目标
            std::vector<module::utils::BoxInfo> targets;
            ///< 当前识别目标
            int curr_label = -1;
            ///< 当前识别目标分数
            double curr_score = 0;
            ///< 当前识别动作
            int elm_label = -1;
            ///< 当前识别动作分数
            double elm_score = 0;
        } extra;

    } cvf_label;

    class CvDetector : public ilsr::CvTask {
    public:
        static CvDetector &get_instance() {
            static CvDetector _det;
            return _det;
        }

        cvf_label get_labels(double overtime = 0.5);

        operation_t get_operation(double overtime = 0.5);

    protected:
        CvDetector();
        CvDetector(const CvDetector &) = delete;
        ~CvDetector();

    protected:
        std::string get_name() {
            return "Detector";
        };

        void process(cv::Mat &frame);

        void on_update(cv::Mat &);

    private:
        // 目标检测器
        nanodet::Nanodet m_detector;
        // 超限学习机
        elm::ELM<9> m_classifier;
        //MatrixXd m_inw;
        //MatrixXd m_bais;
        //MatrixXd m_ouw;
        // SVM
        //cv::Ptr<cv::ml::SVM> m_svm;
        ocsvm::OneClassSVM<8> m_discriminator;

        cvf_label m_result;

        // 条件
        const int MaxHistorySize = 10;
        const double MaxHistoryScore = 0.0;
        std::vector<operation_tag_t> m_operate_history;
    };

}  // namespace recog