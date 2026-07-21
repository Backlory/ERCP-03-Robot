#pragma once
#include "Module/types.h"

namespace net {

    using namespace navi;

    using VideoData = Data<DataType::RgbMap, DataSource::OpenCV, ScalarType::UInt8>;

    //-----------------------------------------------------------------------------

    using DepthData = Data<DataType::ScalarMap, DataSource::OpenCV, ScalarType::Float32>;

    //-----------------------------------------------------------------------------

    using PointsData = Data<DataType::PointClound, DataSource::OpenCV, ScalarType::Float32>;
    using ColorPCData = Data<DataType::ColoredPointClound, DataSource::OpenCV, ScalarType::Float32>;
    using PosedPointData = Data<DataType::PosedPointClound, DataSource::OpenCV, ScalarType::Float32>;


} // namespace net
