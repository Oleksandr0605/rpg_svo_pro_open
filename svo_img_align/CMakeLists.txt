cmake_minimum_required(VERSION 3.10)
project(svo_img_align)

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
        ../svo_direct/include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Source and header files
set(HEADERS_CPU
        include/svo/img_align/sparse_img_align_base.h
        include/svo/img_align/sparse_img_align.h
)

set(SOURCES_CPU
        src/sparse_img_align_base.cpp
        src/sparse_img_align.cpp
)

# Create library
add_library(${PROJECT_NAME} ${SOURCES_CPU} ${HEADERS_CPU})

# Link libraries (Add required libraries here, e.g., OpenCV, Eigen)
set(LINK_LIBS "")  # Set this to the required libraries
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

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
