cmake_minimum_required(VERSION 2.8.12)
project(svo_direct)

# Find OpenCV using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find the libfast.so library
message(STATUS "[1;34mLooking for libfast.so in ${CMAKE_SOURCE_DIR}/../../../../bred/fast/build[0m")
find_library(FAST_LIBRARY NAMES fast PATHS ${CMAKE_SOURCE_DIR}/../../../../bred/fast/build)

find_path(FAST_INCLUDE_DIR NAMES fast PATHS ${CMAKE_SOURCE_DIR}/../../../../bred/fast/include)

if (FAST_LIBRARY AND FAST_INCLUDE_DIR)
    message(STATUS "fast library found: ${FAST_LIBRARY}")
    message(STATUS "fast include directory found: ${FAST_INCLUDE_DIR}")
    set(FAST_LIBRARIES ${FAST_LIBRARY})
    set(FAST_INCLUDE_DIRS ${FAST_INCLUDE_DIR})
else()
    message(FATAL_ERROR "fast not found")
endif()

# Include directories
include_directories(
        include
        ../svo_common/include
        ../../../../bred/minkindr/minkindr/include
        ../vikit/vikit_common/include/
        ../vikit/vikit_cameras/include/
        ../vikit/vikit_solver/include/
        ${FAST_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${FAST_INCLUDE_DIRS}
)

# List of header files
set(HEADERS
        include/svo/direct/patch_warp.h
        include/svo/direct/patch_score.h
        include/svo/direct/patch_utils.h
        include/svo/direct/feature_alignment.h
        include/svo/direct/feature_detection_types.h
        include/svo/direct/feature_detection.h
        include/svo/direct/feature_detection_utils.h
        include/svo/direct/matcher.h
        include/svo/direct/depth_filter.h
        include/svo/direct/depth_estimation.h
        include/svo/direct/elder_zucker.h
)

# List of source files
set(SOURCES
        src/patch_warp.cpp
        src/feature_alignment.cpp
        src/feature_detection.cpp
        src/feature_detection_utils.cpp
        src/matcher.cpp
        src/depth_filter.cpp
        src/depth_estimation.cpp
        src/elder_zucker.cpp
)

# Add the library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} Eigen3::Eigen ${FAST_LIBRARIES})

install(DIRECTORY include/svo/
        DESTINATION include/svo
)

# Export package
export(TARGETS ${PROJECT_NAME} FILE svo_direct-config.cmake)
