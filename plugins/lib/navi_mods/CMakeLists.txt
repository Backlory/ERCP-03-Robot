set(PROJECT_NAME navi_mods)

################################################################################
# Source groups
################################################################################
set(Source
    "Module/modules.cpp"
    "Module/pipeline.cpp"
)

set(ALL_FILES
    ${Source}
)

################################################################################
# Target
################################################################################
add_library(${PROJECT_NAME} STATIC ${ALL_FILES})
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