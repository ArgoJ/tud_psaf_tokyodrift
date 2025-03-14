cmake_minimum_required(VERSION 3.8)
project(button_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(psaf_ucbridge_msgs REQUIRED)


# Add executable
add_executable(
    button_launch 
    src/button_launch_node.cpp
)

# Include directories
include_directories(
    include
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
    rclcpp
    std_msgs
    psaf_ucbridge_msgs
)

ament_target_dependencies(button_launch ${COMMON_DEPENDENCIES})

# Install headers
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
    TARGETS button_launch
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/button_launch
)

if(BUILD_TESTING)
endif()

ament_package()
