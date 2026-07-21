set(PROJECT_NAME remote_cpp)

################################################################################
# Source groups
################################################################################
if(${USE_CODEC_CUDA})
    set(codec
        "../codec/src/decoder.cpp"
        "../codec/src/encoder.cpp"
        "../codec/src/ffmpeg_decoder.cpp"
        "../codec/src/ffmpeg_encoder.cpp"
        "../codec/src/opencv_decoder.cpp"
        "../codec/src/opencv_encoder.cpp"
        "../codec/src/nvidia_encoder.cpp"
        "../codec/src/nvidia_decoder.cpp"
        "../codec/src/opencv_decoder_cuda.cpp"
        "../codec/src/opencv_encoder_cuda.cpp"
        "../codec/src/videolist.cpp"
    )
    set(nvcodec
        "../codec/3rdparty/nvcodec/src/NvDecoder.cpp"
        "../codec/3rdparty/nvcodec/src/NvEncoder.cpp"
        "../codec/3rdparty/nvcodec/src/NvEncoderCuda.cpp"
    )
else()
    set(codec
        "../codec/src/decoder.cpp"
        "../codec/src/encoder.cpp"
        "../codec/src/ffmpeg_decoder.cpp"
        "../codec/src/ffmpeg_encoder.cpp"
        "../codec/src/opencv_decoder.cpp"
        "../codec/src/opencv_encoder.cpp"
        "../codec/src/videolist.cpp"
    )
endif()

set(ALL_FILES
    ${codec}
    ${nvcodec}
    # ${src}
)

################################################################################
# Target
################################################################################
add_library(${PROJECT_NAME} STATIC ${ALL_FILES})
include(${SolutionDir}/cmake/macros.cmake)
include(${SolutionDir}/cmake/vision.cmake)
include(${SolutionDir}/cmake/net.cmake)

################################################################################
# Compile definitions
################################################################################
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        _CRT_SECURE_NO_WARNINGS
    PUBLIC
        $<$<CONFIG:Debug>:DEBUG>
        $<$<CONFIG:RelWithDebInfo>:DEBUG>
        $<$<CONFIG:Release>:NDEBUG>
        $<$<CONFIG:MinSizeRel>:NDEBUG>
)
if(${USE_CODEC_CUDA})
    add_compile_definitions(CODEC_CUDA)
endif()

include_directories(
    ${Solution_INCLUDE}
    ${Vision_INCLUDE}
    include
    ${SolutionDir}/include/include
    ${SolutionDir}/plugins/codec/include
    ${SolutionDir}/plugins/codec/3rdparty/nvcodec/include
    ${SolutionDir}/plugins/drivers/3rdparty/loguru/include/loguru
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${Solution_LIBRARY}
    ${Vision_LIBRARY}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)