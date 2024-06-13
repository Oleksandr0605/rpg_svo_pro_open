cmake_minimum_required(VERSION 3.10)
project(svo_global_map)

set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)


# Find GTSAM
find_package(GTSAM REQUIRED)
message(STATUS "GTSAM include directory: ${GTSAM_INCLUDE_DIR}")
message(STATUS "GTSAM libraries: ${GTSAM_LIBRARIES}")

# Include directories
include_directories(${GTSAM_INCLUDE_DIR})
include_directories(include
        ../svo_common/include
        ../rpg_common/include
        ../vikit/vikit_common/include/
        ../vikit/vikit_cameras/include/
        ../vikit/vikit_solver/include/
        ../../../../bred/minkindr/minkindr/include
        ${OpenCV_INCLUDE_DIRS}
)

# Source and header files
set(HEADERS
        include/svo/gtsam/gtsam_optimizer.h
        include/svo/gtsam/graph_manager.h
        include/svo/gtsam/smart_factors_fwd.h
        include/svo/gtsam/camera_bearing_factor.h
        include/svo/gtsam/camera_bearing_extrinsics_factor.h
        include/svo/global_map.h
        include/svo/null_stream.h
)

set(SOURCES
        src/gtsam/gtsam_optimizer.cpp
        src/gtsam/graph_manager.cpp
        src/global_map.cpp
        src/null_stream.cpp
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} gtsam)

# Installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/svo/
        DESTINATION include/svo
        FILES_MATCHING PATTERN "*.h"
)

# Export
export(TARGETS ${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
)

export(PACKAGE ${PROJECT_NAME})
