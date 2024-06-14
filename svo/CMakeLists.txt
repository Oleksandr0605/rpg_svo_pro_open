cmake_minimum_required(VERSION 3.10)
project(svo)

set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Yaml-cpp REQUIRED)

# Include directories
include_directories(
        include
        ../svo_common/include
        ../../../../bred/minkindr/minkindr/include
        ../../../../bred/fast/include/
        ../vikit/vikit_common/include/
        ../vikit/vikit_cameras/include/
        ../vikit/vikit_solver/include/
        ../svo_direct/include
        ../svo_tracker/include
        ../rpg_common/include
        ../svo_img_align/include
        ../svo_vio_common/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

# Source and header files
set(SVO_SOURCEFILES
        src/frame_handler_mono.cpp
        src/frame_handler_stereo.cpp
        src/frame_handler_array.cpp
        src/frame_handler_base.cpp
        src/map.cpp
        src/pose_optimizer.cpp
        src/initialization.cpp
        src/reprojector.cpp
        src/imu_handler.cpp
        src/stereo_triangulation.cpp
        src/io.cpp
)

set(SVO_HEADERFILES
        include/svo/abstract_bundle_adjustment.h
        include/svo/frame_handler_base.h
        include/svo/frame_handler_mono.h
        include/svo/frame_handler_stereo.h
        include/svo/frame_handler_array.h
        include/svo/global.h
        include/svo/imu_handler.h
        include/svo/initialization.h
        include/svo/map.h
        include/svo/pose_optimizer.h
        include/svo/reprojector.h
        include/svo/stereo_triangulation.h
        include/svo/io.h
        include/svo/svo.h
)

# Create svo library
add_library(svo SHARED ${SVO_SOURCEFILES} ${SVO_HEADERFILES})
target_link_libraries(svo ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})


install(DIRECTORY include/svo/
        DESTINATION include/svo
        FILES_MATCHING PATTERN "*.h"
)

# Export
export(TARGETS svo
        FILE ${PROJECT_NAME}Targets.cmake
)

export(PACKAGE ${PROJECT_NAME})
