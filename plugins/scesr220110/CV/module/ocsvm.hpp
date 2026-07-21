#pragma once
#include <string>
#include <iostream>
#include "../libsvm/svm.h"

namespace ocsvm {

    template <int D>
    class OneClassSVM {
    public:
        OneClassSVM() = default;
        OneClassSVM(const OneClassSVM &) = delete;

        ~OneClassSVM() {
            if (m_model) free(m_model);
        }

    public:
        bool load(const std::string &path) {
            try {
                auto model = svm_load_model(path.c_str());
                if (model) m_model = model;
                return true;
            } catch (std::exception &e) { std::cout << e.what() << std::endl; }
            return false;
        }

        bool is_load() {
            return m_model == NULL;
        }

        double inference(const std::array<double, D> src) {

            // need one more for index = -1
            svm_node x[D + 1];

            for (int i = 0; i < D; ++i) {
                x[i].index = i + 1;
                x[i].value = src[i];
            }
            x[D].index = -1;

            return svm_predict(m_model, x);
        }

    private:
        svm_model *m_model = NULL;
    };

}  // namespace ocsvm
