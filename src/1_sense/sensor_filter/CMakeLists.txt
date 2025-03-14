cmake_minimum_required(VERSION 3.8)
project(sensor_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(utility REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)
find_package(ament_cmake_gtest REQUIRED)


# Add executable
add_executable(
    sensor_filter 
    src/sensor_filter_node.cpp
    src/sensor_filter_config.cpp
)

# Include directories
include_directories(
    include
    ${utility_INCLUDE_DIRS}
    ${helpers_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
    rclcpp
    std_msgs
    sensor_msgs
    utility
    helpers
    timer
)

ament_target_dependencies(sensor_filter ${COMMON_DEPENDENCIES})

# Install headers
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
    TARGETS sensor_filter
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/sensor_filter
)

# OPTIONS
option(TIMEIT "Enable timeit features" ON)

# Define TIMEIT macro for timing purposes
if(TIMEIT)
  add_definitions(-DTIMEIT) 
endif()

if(BUILD_TESTING)
endif()

ament_package()
