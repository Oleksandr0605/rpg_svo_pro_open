cmake_minimum_required(VERSION 3.10)
project(vikit_common)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Set include directories
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ../../../../../bred/minkindr/minkindr/include
)

# Define header and source files
set(HEADERS
        include/vikit/blender_utils.h
        include/vikit/math_utils.h
        include/vikit/performance_monitor.h
        include/vikit/ringbuffer.h
        include/vikit/sample.h
        include/vikit/timer.h
        include/vikit/user_input_thread.h
        include/vikit/vision.h
        include/vikit/homography_decomp.h
        include/vikit/homography.h
        include/vikit/path_utils.h
        include/vikit/csv_utils.h
        include/aslam/common/entrypoint.h
        include/aslam/common/macros.h
        include/aslam/common/memory.h
        include/aslam/common/pose-types.h
        include/aslam/common/yaml-serialization.h
        include/aslam/common/yaml-serialization-eigen.h
)

set(SOURCES
        src/homography.cpp
        src/math_utils.cpp
        src/performance_monitor.cpp
        src/sample.cpp
        src/user_input_thread.cpp
        src/vision.cpp
)

# Create vikit shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/vikit/
        DESTINATION include/vikit
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY include/aslam/
        DESTINATION include/aslam
        FILES_MATCHING PATTERN "*.h")

# Export configuration
export(PACKAGE ${PROJECT_NAME})
