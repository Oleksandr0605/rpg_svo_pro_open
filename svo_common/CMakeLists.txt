cmake_minimum_required(VERSION 3.10)
project(svo_common)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include directories
include_directories(include
        ${OpenCV_INCLUDE_DIRS}
        ../vikit/vikit_common/include/
        ../vikit/vikit_cameras/include/
        ../vikit/vikit_solver/include/
        ../../../../bred/minkindr/minkindr/include
        ../../../../bred/fast/include/
)

# Source and header files
set(HEADERS
        include/svo/common/camera.h
        include/svo/common/camera_fwd.h
        include/svo/common/feature_wrapper.h
        include/svo/common/frame.h
        include/svo/common/imu_calibration.h
        include/svo/common/logging.h
        include/svo/common/occupancy_grid_2d.h
        include/svo/common/point.h
        include/svo/common/seed.h
        include/svo/common/transformation.h
        include/svo/common/types.h
)

set(SOURCES
        src/frame.cpp
        src/point.cpp
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
)

# Export
export(TARGETS ${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
)

export(PACKAGE ${PROJECT_NAME})
