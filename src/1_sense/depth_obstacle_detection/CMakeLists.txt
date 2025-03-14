cmake_minimum_required(VERSION 3.8)
project(depth_obstacle_detection)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(utility REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)
find_package(OpenMP REQUIRED)

# Add executable
add_executable(
    depth_obstacle_detection 
    src/depth_obstacle_detection_node.cpp
)

# Include directories
include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${utility_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(depth_obstacle_detection
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

# Declare dependencies
set(COMMON_DEPENDENCIES 
    rclcpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
    pcl_conversions 
    utility
    helpers
    timer
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math")
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

ament_target_dependencies(depth_obstacle_detection ${COMMON_DEPENDENCIES})

# Install headers
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
    TARGETS depth_obstacle_detection
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/depth_obstacle_detection
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
