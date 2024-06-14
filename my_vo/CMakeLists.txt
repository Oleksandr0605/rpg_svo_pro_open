cmake_minimum_required(VERSION 2.8.12)
project(svo_my_test)

set(CMAKE_PREFIX_PATH "/usr/local" "/usr")

# Find the required packages
find_package(yaml-cpp REQUIRED)

find_package(OpenCV CONFIG REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find GTSAM
find_package(GTSAM REQUIRED)
message(STATUS "GTSAM include directory: ${GTSAM_INCLUDE_DIR}")

find_library(GLOG_LIBRARY glog)
find_path(GLOG_INCLUDE_DIR glog/logging.h)

if (GLOG_LIBRARY AND GLOG_INCLUDE_DIR)
    set(GLOG_LIBRARIES ${GLOG_LIBRARY})
    set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
else()
    message(FATAL_ERROR "glog not found")
endif()

# Find the custom libsvo.so library
message(STATUS "[1;34mLooking for libsvo.so in ${CMAKE_SOURCE_DIR}/../svo/build[0m")
find_library(SVO_LIBRARY NAMES svo PATHS ${CMAKE_SOURCE_DIR}/../svo/build)

find_path(SVO_INCLUDE_DIR NAMES svo PATHS ${CMAKE_SOURCE_DIR}/../svo/include)

if (SVO_LIBRARY AND SVO_INCLUDE_DIR)
    message(STATUS "libsvo library found: ${SVO_LIBRARY}")
    message(STATUS "libsvo include directory found: ${SVO_INCLUDE_DIR}")
    set(SVO_LIBRARIES ${SVO_LIBRARY})
    set(SVO_INCLUDE_DIRS ${SVO_INCLUDE_DIR})
else()
    message(FATAL_ERROR "libsvo not found")
endif()

# Find the libvikit_common.so library
message(STATUS "[1;34mLooking for libvikit_common.so in ${CMAKE_SOURCE_DIR}/../vikit/vikit_common/build[0m")
find_library(VIKIT_COMMON_LIBRARY NAMES vikit_common PATHS ${CMAKE_SOURCE_DIR}/../vikit/vikit_common/build)

find_path(VIKIT_COMMON_INCLUDE_DIR NAMES vikit PATHS ${CMAKE_SOURCE_DIR}/../vikit/vikit_common/include)

if (VIKIT_COMMON_LIBRARY AND VIKIT_COMMON_INCLUDE_DIR)
    message(STATUS "libvikit_common library found: ${VIKIT_COMMON_LIBRARY}")
    message(STATUS "libvikit_common include directory found: ${VIKIT_COMMON_INCLUDE_DIR}")
    set(VIKIT_COMMON_LIBRARIES ${VIKIT_COMMON_LIBRARY})
    set(VIKIT_COMMON_INCLUDE_DIRS ${VIKIT_COMMON_INCLUDE_DIR})
else()
    message(FATAL_ERROR "libvikit_common not found in ${CMAKE_SOURCE_DIR}/../vikit/vikit_common/build")
endif()

# Find the libvikit_solver.so library
message(STATUS "[1;34mLooking for libvikit_solver.so in ${CMAKE_SOURCE_DIR}/../vikit/vikit_solver/build[0m")
find_library(VIKIT_SOLVER_LIBRARY NAMES vikit_solver PATHS ${CMAKE_SOURCE_DIR}/../vikit/vikit_solver/build)

find_path(VIKIT_SOLVER_INCLUDE_DIR NAMES vikit PATHS ${CMAKE_SOURCE_DIR}/../vikit/vikit_solver/include)

if (VIKIT_SOLVER_LIBRARY AND VIKIT_SOLVER_INCLUDE_DIR)
    message(STATUS "libvikit_solver library found: ${VIKIT_SOLVER_LIBRARY}")
    message(STATUS "libvikit_solver include directory found: ${VIKIT_SOLVER_INCLUDE_DIR}")
    set(VIKIT_SOLVER_LIBRARIES ${VIKIT_SOLVER_LIBRARY})
    set(VIKIT_SOLVER_INCLUDE_DIRS ${VIKIT_SOLVER_INCLUDE_DIR})
else()
    message(FATAL_ERROR "libvikit_solver not found")
endif()

# Find the libvikit_cameras.a library
message(STATUS "[1;34mLooking for ${CMAKE_SOURCE_DIR}/../vikit/vikit_cameras/build[0m")
find_library(VIKIT_CAMERAS_LIBRARY NAMES vikit_cameras PATHS ${CMAKE_SOURCE_DIR}/../vikit/vikit_cameras/build)

if (VIKIT_CAMERAS_LIBRARY)
    message(STATUS "libvikit_cameras library found: ${VIKIT_CAMERAS_LIBRARY}")
    set(VIKIT_CAMERAS_LIBRARIES ${VIKIT_CAMERAS_LIBRARY})
else()
    message(FATAL_ERROR "not found ${CMAKE_SOURCE_DIR}/../vikit/vikit_cameras/build")
endif()

# Find the libsvo_common.a library
message(STATUS "[1;34mLooking for ${CMAKE_SOURCE_DIR}/../svo_common/build[0m")
find_library(SVO_COMMON_LIBRARY NAMES svo_common PATHS ${CMAKE_SOURCE_DIR}/../svo_common/build)

if (SVO_COMMON_LIBRARY)
    message(STATUS "libsvo_common library found: ${SVO_COMMON_LIBRARY}")
    set(SVO_COMMON_LIBRARIES ${SVO_COMMON_LIBRARY})
else()
    message(FATAL_ERROR "not found ${CMAKE_SOURCE_DIR}/../svo_common/build")
endif()

# Find the libsvo_img_align.a library
message(STATUS "[1;34mLooking for ${CMAKE_SOURCE_DIR}/../svo_img_align/build[0m")
find_library(SVO_IMG_ALIGN_LIBRARY NAMES svo_img_align PATHS ${CMAKE_SOURCE_DIR}/../svo_img_align/build)

if (SVO_IMG_ALIGN_LIBRARY)
    message(STATUS "libsvo_img_align library found: ${SVO_IMG_ALIGN_LIBRARY}")
    set(SVO_IMG_ALIGN_LIBRARIES ${SVO_IMG_ALIGN_LIBRARY})
else()
    message(FATAL_ERROR "not found ${CMAKE_SOURCE_DIR}/../svo_img_align/build")
endif()

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

# Find the libsvo_direct.a library
message(STATUS "[1;34mLooking for ${CMAKE_SOURCE_DIR}/../svo_direct/build[0m")
find_library(SVO_DIRECT_LIBRARY NAMES svo_direct PATHS ${CMAKE_SOURCE_DIR}/../svo_direct/build)

if (SVO_DIRECT_LIBRARY)
    message(STATUS "libsvo_direct library found: ${SVO_DIRECT_LIBRARY}")
    set(SVO_DIRECT_LIBRARIES ${SVO_DIRECT_LIBRARY})
else()
    message(FATAL_ERROR "not found ${CMAKE_SOURCE_DIR}/../svo_direct/build")
endif()

# Find the libsvo_tracker.a library
message(STATUS "[1;34mLooking for ${CMAKE_SOURCE_DIR}/../svo_tracker/build[0m")
find_library(SVO_TRACKER_LIBRARY NAMES svo_tracker PATHS ${CMAKE_SOURCE_DIR}/../svo_tracker/build)

if (SVO_TRACKER_LIBRARY)
    message(STATUS "libsvo_tracker library found: ${SVO_TRACKER_LIBRARY}")
    set(SVO_TRACKER_LIBRARIES ${SVO_TRACKER_LIBRARY})
else()
    message(FATAL_ERROR "not found ${CMAKE_SOURCE_DIR}/../svo_tracker/build")
endif()

# Include directories
include_directories(
        include
        ../svo_common/include
        ../../../../bred/minkindr/minkindr/include
        ../../../../bred/fast/include/
        ../vikit/vikit_cameras/include/
        ../rpg_common/include/
        ../svo_direct/include/
        ../svo_ros/include/
        ${OpenCV_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIRS}
        ${SVO_INCLUDE_DIRS}
        ${VIKIT_COMMON_INCLUDE_DIRS}
        ${VIKIT_SOLVER_INCLUDE_DIRS}
        ${SVO_IMG_ALIGN_INCLUDE_DIRS}
        ${FAST_INCLUDE_DIRS}
        ${SVO_DIRECT_INCLUDE_DIRS}
        ${SVO_TRACKER_INCLUDE_DIRS}
        ${VIKIT_CAMERAS_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
)

# List of source files
set(SOURCES
        vo_test.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${GTSAM_LIBRARIES}
        ${GTEST_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${SVO_LIBRARIES}
        ${VIKIT_COMMON_LIBRARIES}
        ${VIKIT_SOLVER_LIBRARIES}
        ${SVO_COMMON_LIBRARIES}
        ${SVO_IMG_ALIGN_LIBRARIES}
        ${SVO_DIRECT_LIBRARIES}
        ${FAST_LIBRARIES}
        ${SVO_TRACKER_LIBRARIES}
        ${VIKIT_CAMERAS_LIBRARIES}
        yaml-cpp
        pthread
)

install(DIRECTORY include/svo/
        DESTINATION include/svo
)
