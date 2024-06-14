cmake_minimum_required(VERSION 3.10)
project(svo_vio_common)

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
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Source and header files
set(HEADERS
        include/svo/vio_common/logging.hpp
        include/svo/vio_common/matrix_operations.hpp
        include/svo/vio_common/matrix.hpp
        include/svo/vio_common/test_utils.hpp
        include/svo/vio_common/backend_types.hpp
)

set(SOURCES
        src/vio_common.cpp
        src/test_utils.cpp
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
        FILES_MATCHING PATTERN "*.hpp"
)

# Export
export(TARGETS ${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
)

export(PACKAGE ${PROJECT_NAME})
