cmake_minimum_required(VERSION 3.10)
project(svo_pgo)

set(CMAKE_CXX_STANDARD 14)

# Find OpenCV using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include directories
include_directories(include
        ../svo_common/include
        ../../../../bred/minkindr/minkindr/include
        ../../../../bred/fast/include/
        ../vikit/vikit_common/include/
        ../vikit/vikit_cameras/include/
        ../vikit/vikit_solver/include/
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Source and header files
set(HEADERS
        include/svo/pgo/pgo.h
)

set(SOURCES
        src/pgo.cpp
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})


install(DIRECTORY include/svo/
        DESTINATION include/svo
        FILES_MATCHING PATTERN "*.h"
)

# Export
export(TARGETS ${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
)

export(PACKAGE ${PROJECT_NAME})
