cmake_minimum_required(VERSION 3.8)
project(bezier_curve)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(utility REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)


# Add executable
add_executable(
    bezier_curve 
    src/bezier_curve_node.cpp 
    src/bezier_curve_config.cpp
)

# Include directories
include_directories(
    include
    ${utility_INCLUDE_DIRS}
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
    rclcpp
    geometry_msgs
    visualization_msgs
    utility
    helpers
    timer
)

ament_target_dependencies(bezier_curve ${COMMON_DEPENDENCIES})

# Install headers
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
    TARGETS bezier_curve
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/bezier_curve
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

