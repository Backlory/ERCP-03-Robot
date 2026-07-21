cmake_minimum_required(VERSION 3.0.0)
project(ercp2)

SET(CMAKE_C_COMPILER "/usr/bin/clang")
SET(CMAKE_CXX_COMPILER "/usr/bin/clang++")

set(CMAKE_BUILD_TYPE "debug")
set(CMAKE_INSTALL_PREFIX "/home/ilsr/opt")
set(CMAKE_CXX_STANDARD 17)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

set(SolutionDir ${PROJECT_SOURCE_DIR})
set(USE_CODEC_CUDA OFF)

find_package(Boost REQUIRED COMPONENTS atomic chrono filesystem serialization thread)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)
find_package(fmt REQUIRED)

set(Solution_INCLUDE
    ${Boost_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
    ${SolutionDir}/include/include
    ${SolutionDir}/plugins/drivers/3rdparty
    ${SolutionDir}/plugins/drivers/3rdparty/beauty/include
    # ${SolutionDir}/plugins/library/3rdparty/fmt-7.1.3/include
    ${SolutionDir}/plugins/drivers/3rdparty/loguru/include
    ${SolutionDir}/plugins/drivers/dep/include
)

set(Solution_LIBRARY
    ${Boost_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    TBB::tbb
    fmt::fmt
    Eigen3::Eigen
    ${CMAKE_DL_LIBS}
)

message(STATUS "Solution folder: ${SolutionDir}")
message(STATUS "    include: ${Solution_INCLUDE}")
message(STATUS "    lib: ${Solution_LIBRARY}")

add_subdirectory(${SolutionDir}/plugins/library)
add_subdirectory(${SolutionDir}/plugins/navi_mods)
add_subdirectory(${SolutionDir}/plugins/net_mods)
add_subdirectory(${SolutionDir}/plugins/remote)
add_subdirectory(${SolutionDir}/NavigateSystem/)

add_subdirectory(${SolutionDir}/plugins/endodepth)
