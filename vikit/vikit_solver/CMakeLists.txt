cmake_minimum_required(VERSION 3.10)
project(vikit_solver)

# Set build type
set(CMAKE_BUILD_TYPE Release) # Options: Release, RelWithDebInfo

# Set C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")

# Include directories
include_directories(include)

# Define source files
set(SOURCES
        src/robust_cost.cpp
)

set(HEADERS
        include/vikit/solver/mini_least_squares_solver.h
        include/vikit/solver/implementation/mini_least_squares_solver.hpp
        include/vikit/solver/robust_cost.h
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

## Declare a cpp executable (uncomment if needed)
# add_executable(elevation_map_node src/elevation_map_node.cpp)
# target_link_libraries(elevation_map_node ${PROJECT_NAME})

##########
# GTESTS #
##########
# Enable testing
enable_testing()

# Find Google Test
find_package(GTest REQUIRED)
include_directories(${GTest_INCLUDE_DIRS})

# Unit tests (uncomment if needed)
# add_executable(test_cameras test/test-cameras.cc)
# target_link_libraries(test_cameras ${GTest_LIBRARIES} ${PROJECT_NAME})
# add_test(NAME test_cameras COMMAND test_cameras)

##########
# EXPORT #
##########

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/vikit/
        DESTINATION include/vikit
        FILES_MATCHING PATTERN "*.h")

# Export configuration
export(PACKAGE ${PROJECT_NAME})
