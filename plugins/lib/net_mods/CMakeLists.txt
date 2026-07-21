set(PROJECT_NAME net_mods)

################################################################################
# Target
################################################################################
add_library(${PROJECT_NAME} STATIC 
    "decoder_impl.cpp" "encoder_impl.cpp")
include(${SolutionDir}/cmake/macros.cmake)
include(${SolutionDir}/cmake/vision.cmake)

################################################################################
# Compile definitions
################################################################################
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        _CRT_SECURE_NO_WARNINGS
    PUBLIC
        $<$<CONFIG:Debug>:DEBUG>
        $<$<CONFIG:RelWithDebInfo>:DEBUG>
        $<$<CONFIG:Release>:NDEBUG>
        $<$<CONFIG:MinSizeRel>:NDEBUG>
)

include_directories(
    ${Solution_INCLUDE}
    ${Vision_INCLUDE}
    ${SolutionDir}/plugins/navi_mods
    ${SolutionDir}/plugins/codec/include
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