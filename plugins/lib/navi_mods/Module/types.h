#pragma once
#include <array>
#include <type_traits>
#include <opencv2/opencv.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/impl/point_types.hpp>
#include <Eigen/Dense>
// #include <boost/numeric/ublas/matrix.hpp>

namespace navi {

    // template<typename T>
    // using bmatrix = boost::numeric::ublas::matrix<T>;

    /**
     * 默认数据格式: (依次为优先级顺序)
     */
    enum class DataType : int {
        /// 2D标量
        ScalarMap,
        /// 2D图像
        RgbMap,
        /// 3D点云
        PointClound,
        /// 3D密度点云
        PointCloundWithDensity,
        /// 3D有色点云
        ColoredPointClound,
        /// 3D姿态点云
        PosedPointClound,
        /// 3D姿态点
        PosedPoint,
    };

    enum class DataSource : int {
        OpenCV,
        PCL,
        Eigen,
        Torch,
        uBlas,
    };

    enum class ScalarType : int {
        UInt8,
        Int8,
        UInt16,
        Int16,
        UInt32,
        Int32,
        UInt64,
        Int64,
        Float32,
        Float64,
    };

    namespace detail {

        template <typename...>
        struct table;

        template <>
        struct table<> {
            using type = void;
        };

        template <typename Item0, typename... Items>
        struct table<Item0, Items...> {
            using type = Item0;
        };

        ////////////////////////////////////////////////////////////////

        template <DataType DT, DataSource DS, ScalarType ST, typename T>
        struct type_item {
            static constexpr DataType data_type = DT;
            static constexpr DataSource data_source = DS;
            static constexpr ScalarType scalar_type = ST;
            using type = T;
        };

        // 根据数据类型和来源推断存储类型
        using t = DataType;
        using s = DataSource;
        using d = ScalarType;
        // clang-format off
        using type_table = table<
            //  -----------+--------------------+-------------------+--------------------+-----------+-----------------+-
              type_item< t::ScalarMap,                  s::OpenCV,     d::Float64,        cv::Mat1d  >, //
              type_item< t::ScalarMap,                  s::OpenCV,     d::Float32,        cv::Mat1f  >, //
              type_item< t::ScalarMap,                  s::OpenCV,     d::UInt16,         cv::Mat1w  >, //
              type_item< t::ScalarMap,                  s::OpenCV,     d::UInt8,          cv::Mat1b  >, //
            //type_item< t::ScalarMap,                  s::PCL,        d::Float64,        pcl::PointXYZ >, //
            //type_item< t::ScalarMap,                  s::PCL,        d::Float32,        pcl::PointXYZ >, //
            //type_item< t::ScalarMap,                  s::PCL,        d::UInt8,          pcl::PointXYZ >, //
              type_item< t::ScalarMap,                  s::Eigen,      d::Float64,        Eigen::MatrixXd >, //
              type_item< t::ScalarMap,                  s::Eigen,      d::Float32,        Eigen::MatrixXf >, //
              type_item< t::ScalarMap,                  s::Eigen,      d::UInt8,          Eigen::Matrix<uint8_t, -1, -1> >, //
            //type_item< t::ScalarMap,                  s::Torch,      d::Float64,        torch::Tensor >, //
            //type_item< t::ScalarMap,                  s::Torch,      d::Float32,        torch::Tensor >, //
            //type_item< t::ScalarMap,                  s::Torch,      d::UInt8,          torch::Tensor >, //
          
              type_item< t::RgbMap,                     s::OpenCV,     d::Float64,        cv::Mat3d >, //
              type_item< t::RgbMap,                     s::OpenCV,     d::Float32,        cv::Mat3f >, //
              type_item< t::RgbMap,                     s::OpenCV,     d::UInt8,          cv::Mat3b >, //
            //type_item< t::RgbMap,                     s::PCL,        d::Float64,        >, //
            //type_item< t::RgbMap,                     s::PCL,        d::Float32,        >, //
            //type_item< t::RgbMap,                     s::PCL,        d::UInt8,          >, //
            //type_item< t::RgbMap,                     s::Eigen,      d::Float64,        std::array<Eigen::MatrixXd, 3> >, //
            //type_item< t::RgbMap,                     s::Eigen,      d::Float32,        std::array<Eigen::MatrixXf, 3> >, //
            //type_item< t::RgbMap,                     s::Eigen,      d::UInt8,          std::array<Eigen::Matrix<uint8_t, -1, -1>, 3> >, //
            //type_item< t::RgbMap,                     s::Torch,      d::Float64,        torch::Tensor >, //
            //type_item< t::RgbMap,                     s::Torch,      d::Float32,        torch::Tensor >, //
            //type_item< t::RgbMap,                     s::Torch,      d::UInt8,          torch::Tensor >, //
        
            //type_item< t::PointClound,                s::OpenCV,     d::Float64,        >, //
            //type_item< t::PointClound,                s::OpenCV,     d::Float32,        >, //
            //type_item< t::PointClound,                s::PCL,        d::Float64,        pcl::PointCloud<pcl::PointXYZ> >, //
            //type_item< t::PointClound,                s::PCL,        d::Float32,        pcl::PointCloud<pcl::PointXYZ> >, //
              type_item< t::PointClound,                s::Eigen,      d::Float64,        Eigen::MatrixX3d >, //
              type_item< t::PointClound,                s::Eigen,      d::Float32,        Eigen::MatrixX3f >, //
            //type_item< t::PointClound,                s::Torch,      d::Float64,        torch::Tensor >, //
            //type_item< t::PointClound,                s::Torch,      d::Float32,        torch::Tensor >, //
        
            //type_item< t::PointCloundWithDensity,     s::OpenCV,     d::Float64,        >, //
            //type_item< t::PointCloundWithDensity,     s::OpenCV,     d::Float32,        >, //
            //type_item< t::PointCloundWithDensity,     s::PCL,        d::Float64,        pcl::PointCloud<pcl::PointXYZI> >, //
            //type_item< t::PointCloundWithDensity,     s::PCL,        d::Float32,        pcl::PointCloud<pcl::PointXYZI> >, //
              type_item< t::PointCloundWithDensity,     s::Eigen,      d::Float64,        Eigen::MatrixX4d >, //
              type_item< t::PointCloundWithDensity,     s::Eigen,      d::Float32,        Eigen::MatrixX4f >, //
            //type_item< t::PointCloundWithDensity,     s::Torch,      d::Float64,        torch::Tensor >, //
            //type_item< t::PointCloundWithDensity,     s::Torch,      d::Float32,        torch::Tensor >, //
        
              type_item< t::ColoredPointClound,         s::OpenCV,     d::Float64,        cv::Mat1d>, //
              type_item< t::ColoredPointClound,         s::OpenCV,     d::Float32,        cv::Mat1f>, //
            //type_item< t::ColoredPointClound,         s::PCL,        d::Float64,        pcl::PointCloud<pcl::PointXYZRGB> >, //
            //type_item< t::ColoredPointClound,         s::PCL,        d::Float32,        pcl::PointCloud<pcl::PointXYZRGB> >, //
            //type_item< t::ColoredPointClound,         s::Eigen,      d::Float64,        Eigen::Matrix<double, 6, -1> >, //
            //type_item< t::ColoredPointClound,         s::Eigen,      d::Float32,        Eigen::Matrix<float, 6, -1> >, //
            //type_item< t::ColoredPointClound,         s::Torch,      d::Float64,        torch::Tensor >, //
            //type_item< t::ColoredPointClound,         s::Torch,      d::Float32,        torch::Tensor >, //
        
              type_item< t::PosedPointClound,           s::OpenCV,     d::Float64,        cv::Mat1d>, //
              type_item< t::PosedPointClound,           s::OpenCV,     d::Float32,        cv::Mat1f>, //
            //type_item< t::PosedPointClound,           s::PCL,        d::Float64,        pcl::PointCloud<pcl::PointNormal> >, //
            //type_item< t::PosedPointClound,           s::PCL,        d::Float32,        pcl::PointCloud<pcl::PointNormal> >, //
            //type_item< t::PosedPointClound,           s::Eigen,      d::Float64,        Eigen::Matrix<double, 6, -1> >, //
            //type_item< t::PosedPointClound,           s::Eigen,      d::Float32,        Eigen::Matrix<float, 6, -1> >, //
            //type_item< t::PosedPointClound,           s::Torch,      d::Float64,        torch::Tensor >, //
            //type_item< t::PosedPointClound,           s::Torch,      d::Float32,        torch::Tensor >  //
            
            //type_item< t::PosedPoint,                 s::OpenCV,     d::Float64,        >, //
            //type_item< t::PosedPoint,                 s::OpenCV,     d::Float32,        >, //
            //type_item< t::PosedPoint,                 s::PCL,        d::Float64,        pcl::PointNormal>, //
            //type_item< t::PosedPoint,                 s::PCL,        d::Float32,        pcl::PointNormal>, //
              type_item< t::PosedPoint,                 s::Eigen,      d::Float64,        Eigen::Matrix<double, 6, 1> >, //
              type_item< t::PosedPoint,                 s::Eigen,      d::Float32,        Eigen::Matrix<float, 6, 1> > //
            //type_item< t::PosedPoint,                 s::Torch,      d::Float64,        torch::Tensor >, //
            //type_item< t::PosedPoint,                 s::Torch,      d::Float32,        torch::Tensor >  //
            //  -----------+--------------------+-------------------+--------------------+-----------+-----------------+-
        >;
        // clang-format on

        ////////////////////////////////////////////////////////////////

        template <DataType DT, DataSource DS, ScalarType ST, typename...>
        struct filter;

        template <DataType DT, DataSource DS, ScalarType ST>
        struct filter<DT, DS, ST, table<>> {
            using type = typename table<>::type;
        };

        template <DataType DT, DataSource DS, ScalarType ST, typename Item0, typename... Items>
        struct filter<DT, DS, ST, table<Item0, Items...>> {
            using S1 = typename filter<DT, DS, ST, table<Items...>>::type;
            using T2 = typename std::conditional<Item0::scalar_type == ST, Item0, S1>::type;
            using T3 = typename std::conditional<Item0::data_source == DS, T2, S1>::type;
            using type = typename std::conditional<Item0::data_type == DT, T3, S1>::type;
        };

    } // namespace detail

    template <DataType DT, DataSource DS, ScalarType ST>
    using Data = typename detail::filter<DT, DS, ST, detail::type_table>::type;

    template <DataType DT, DataSource DS, ScalarType ST>
    using Value = typename Data<DT, DS, ST>::type;

    // 同类/同源/异型数据转换(类型转换)
    template <DataType DT, DataSource DS, ScalarType S1, ScalarType S2>
    struct SConverter {
        using left_type = typename detail::filter<DT, DS, S1, detail::type_table>::type::type;
        using right_type = typename detail::filter<DT, DS, S2, detail::type_table>::type::type;

        static void convert(const left_type &src, right_type &dst);
    };

    // 同类/异源/同型数据转换(数据转换)
    template <DataType DT, DataSource DS1, DataSource DS2, ScalarType S>
    struct DSConverter {
        using left_type = typename detail::filter<DT, DS1, S, detail::type_table>::type::type;
        using right_type = typename detail::filter<DT, DS2, S, detail::type_table>::type::type;

        static void convert(const left_type &src, right_type &dst);
    };

    // 同类/异源/异型数据转换(类型转换 + 数据转换)
    template <DataType DT, DataSource DS1, DataSource DS2, ScalarType S1, ScalarType S2>
    struct DSSConverter {
        using left_type = typename detail::filter<DT, DS1, S1, detail::type_table>::type::type;
        using right_type = typename detail::filter<DT, DS2, S2, detail::type_table>::type::type;

        static void convert(const left_type &src, right_type &dst);
    };

} // namespace navi
