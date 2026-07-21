set(PROJECT_NAME library)

################################################################################
# Source groups
################################################################################
set(Source
    "${SolutionDir}/plugins/drivers/3rdparty/beauty/src/session.cpp"
    "${SolutionDir}/plugins/drivers/3rdparty/loguru/src/loguru.cpp"
    "${SolutionDir}/plugins/drivers/dep/src/utils.cpp"
)

set(ALL_FILES
    ${Source}
)

################################################################################
# Target
################################################################################
add_library(${PROJECT_NAME} STATIC ${ALL_FILES})
include(${SolutionDir}/cmake/macros.cmake)

################################################################################
# Includes for CMake
################################################################################
set(ROOT_NAMESPACE library)

################################################################################
# Compile definitions
################################################################################
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        ROBOT_EXPORT
        _CRT_SECURE_NO_WARNINGS
    PUBLIC
        $<$<CONFIG:Debug>:DEBUG>
        $<$<CONFIG:RelWithDebInfo>:DEBUG>
        $<$<CONFIG:Release>:NDEBUG>
        $<$<CONFIG:MinSizeRel>:NDEBUG>
)

include_directories(
    ${Solution_INCLUDE}
    ${SolutionDir}/plugins/drivers/3rdparty/loguru/include/loguru
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${Solution_LIBRARY}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)