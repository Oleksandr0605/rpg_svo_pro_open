cmake_minimum_required(VERSION 3.10)
project(svo_tracker)

set(CMAKE_CXX_STANDARD 14)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

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
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Source and header files
set(HEADERS
        include/svo/tracker/feature_tracking_types.h
        include/svo/tracker/feature_tracking_utils.h
        include/svo/tracker/feature_tracking_viz.h
        include/svo/tracker/feature_tracker.h
)

set(SOURCES
        src/feature_tracking_types.cpp
        src/feature_tracking_utils.cpp
        src/feature_tracking_viz.cpp
        src/feature_tracker.cpp
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries to the library
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
)

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
