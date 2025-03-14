cmake_minimum_required(VERSION 3.8)
project(purepursuite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utility REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)



add_executable(
        purepursuite 
        src/purepursuit_control_node.cpp 
        src/longitudinal_control.cpp 
        src/lateral_control.cpp
)

include_directories(
        include
        ${utility_INCLUDE_DIRS}
)

ament_target_dependencies(purepursuite
        rclcpp
        utility
        geometry_msgs
        sensor_msgs
        helpers
        timer
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS purepursuite
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/purepursuite
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
