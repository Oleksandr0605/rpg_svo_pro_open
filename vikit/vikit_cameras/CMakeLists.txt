cmake_minimum_required(VERSION 3.0)
project(vikit_cameras VERSION 1.0.0)  # Add version here

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the required packages
find_package(yaml-cpp REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# Include directories
include_directories(
        include
        ../../vikit/vikit_common/include
        ../../../../../bred/minkindr/minkindr/include
        ${yaml-cpp_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

# Header files
set(HEADERS
        include/vikit/cameras.h
        include/vikit/cameras/camera_geometry_base.h
        include/vikit/cameras/implementation/camera_geometry_base.hpp
        include/vikit/cameras/no_distortion.h
        include/vikit/cameras/atan_distortion.h
        include/vikit/cameras/equidistant_distortion.h
        include/vikit/cameras/equidistant_fisheye_geometry.h
        include/vikit/cameras/equidistant_fisheye_projection.h
        include/vikit/cameras/omni_geometry.h
        include/vikit/cameras/omni_projection.h
        include/vikit/cameras/radial_tangential_distortion.h
        include/vikit/cameras/pinhole_projection.h
        include/vikit/cameras/implementation/pinhole_projection.hpp
        include/vikit/cameras/camera_geometry.h
        include/vikit/cameras/implementation/camera_geometry.hpp
        include/vikit/cameras/camera_factory.h
        include/vikit/cameras/ncamera.h
        include/vikit/cameras/yaml/camera-yaml-serialization.h
        include/vikit/cameras/yaml/ncamera-yaml-serialization.h
)

# Source files
set(SOURCES
        src/camera_factory.cpp
        src/ncamera.cpp
        src/camera_geometry_base.cpp
        src/camera_yaml_serialization.cpp
        src/equidistant_fisheye_geometry.cpp
        src/equidistant_fisheye_projection.cpp
        src/omni_geometry.cpp
        src/omni_projection.cpp
        src/ncamera_yaml_serialization.cpp
)

# Add the library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} yaml-cpp Eigen3::Eigen ${OpenCV_LIBRARIES})

install(DIRECTORY include/vikit
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export the package configuration
include(CMakePackageConfigHelpers)
#write_basic_package_version_file(
#        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#        VERSION ${PROJECT_VERSION}
#        COMPATIBILITY AnyNewerVersion
#)
#
#configure_package_config_file(
#        "${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
#        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#        INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
#)
#
#install(FILES
#        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#        DESTINATION lib/cmake/${PROJECT_NAME}
#)
#
#export(EXPORT ${PROJECT_NAME}Targets
#        FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
#        NAMESPACE ${PROJECT_NAME}::
#)
#
#install(EXPORT ${PROJECT_NAME}Targets
#        FILE ${PROJECT_NAME}Targets.cmake
#        NAMESPACE ${PROJECT_NAME}::
#        DESTINATION lib/cmake/${PROJECT_NAME}
#)
