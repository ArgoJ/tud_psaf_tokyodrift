cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(utility REQUIRED)



# Add executable
add_executable(
        obstacle_detection 
        src/obstacle_detection_node.cpp 
)

# Include directories
include_directories(
        include
        
)




ament_target_dependencies(obstacle_detection 
        rclcpp
        utility
        sensor_msgs
)
# Install headers
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
        TARGETS obstacle_detection
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/obstacle_detection
)

if(BUILD_TESTING)
endif()

ament_package()
