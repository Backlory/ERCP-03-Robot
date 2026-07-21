#pragma once
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/binary_object.hpp>
#include <opencv2/opencv.hpp>

namespace boost {
    namespace serialization {

        // https://www.patrikhuber.ch/files/code/matserialisation.hpp

        /**
         * Serialize a cv::Mat using boost::serialization.
         *
         * Supports all types of matrices as well as non-contiguous ones.
         *
         * @param[in] ar The archive to serialise to (or to serialise from).
         * @param[in] mat The matrix to serialise (or deserialise).
         * @param[in] version An optional version argument.
         */
        template <class Archive>
        void serialize(Archive &ar, cv::Mat &mat, const unsigned int /*version*/)
        {
            int rows, cols, type;
            bool continuous;

            if (Archive::is_saving::value) {
                rows = mat.rows;
                cols = mat.cols;
                type = mat.type();
                continuous = mat.isContinuous();
            }

            ar &BOOST_SERIALIZATION_NVP(rows) & BOOST_SERIALIZATION_NVP(cols)
                & BOOST_SERIALIZATION_NVP(type) & BOOST_SERIALIZATION_NVP(continuous);

            if (Archive::is_loading::value)
                mat.create(rows, cols, type);

            if (continuous) {
                const int data_size = rows * cols * static_cast<int>(mat.elemSize());
                boost::serialization::binary_object mat_data(mat.data, data_size);
                ar &BOOST_SERIALIZATION_NVP(mat_data);
            } else {
                const int row_size = cols * static_cast<int>(mat.elemSize());
                for (int i = 0; i < rows; i++) {
                    boost::serialization::binary_object row_data(mat.ptr(i), row_size);
                    std::string row_name("mat_data_row_" + std::to_string(i));
                    ar &make_nvp(row_name.c_str(), row_data);
                }
            }
        }

        template <class Archive, class Ty>
        void serialize(Archive &ar, cv::Mat_<Ty> &mat, const unsigned int /*version*/)
        {
            int rows, cols, type;
            bool continuous;

            if (Archive::is_saving::value) {
                rows = mat.rows;
                cols = mat.cols;
                type = mat.type();
                continuous = mat.isContinuous();
            }

            ar &BOOST_SERIALIZATION_NVP(rows) & BOOST_SERIALIZATION_NVP(cols)
                & BOOST_SERIALIZATION_NVP(type) & BOOST_SERIALIZATION_NVP(continuous);

            if (Archive::is_loading::value)
                mat.create(rows, cols);

            if (continuous) {
                const int data_size = rows * cols * static_cast<int>(mat.elemSize());
                boost::serialization::binary_object mat_data(mat.data, data_size);
                ar &BOOST_SERIALIZATION_NVP(mat_data);
            } else {
                const int row_size = cols * static_cast<int>(mat.elemSize());
                for (int i = 0; i < rows; i++) {
                    boost::serialization::binary_object row_data(mat.ptr(i), row_size);
                    std::string row_name("mat_data_row_" + std::to_string(i));
                    ar &make_nvp(row_name.c_str(), row_data);
                }
            }
        }

    } // namespace serialization
} // namespace boost
