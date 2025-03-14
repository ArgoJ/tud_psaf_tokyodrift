cmake_minimum_required(VERSION 3.8)
project(lane_detection)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(utility REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)
find_package(OpenMP REQUIRED)


# Add executable
add_executable(
        lane_detection 
        src/lane_detection_node.cpp 
        src/lane_detection.cpp 
        src/lane_detection_config.cpp
)

# Include directories
include_directories(
        include
        ${utility_INCLUDE_DIRS}
        ${YAML_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(lane_detection
        yaml-cpp
        ${OpenCV_LIBRARIES}
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
        OpenCV
        cv_bridge
        Eigen3
        utility
        helpers
        timer
)

if (OPENMP_FOUND)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

ament_target_dependencies(lane_detection ${COMMON_DEPENDENCIES})

# Install headers
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# install launch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)


# Install executable
install(
        TARGETS lane_detection
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/lane_detection
)

# OPTIONS
option(BUILD_IMAGE_PUBLISHER_TEST "Build image publisher test node" OFF)
option(TIMEIT "Enable timeit features" ON)

# Optional image publisher test node
if(BUILD_IMAGE_PUBLISHER_TEST)
        add_executable(image_publisher src/image_publisher_test.cpp)
        ament_target_dependencies(image_publisher ${COMMON_DEPENDENCIES})
        install(TARGETS image_publisher DESTINATION lib/${PROJECT_NAME})
endif()

# Define TIMEIT macro for timing purposes
if(TIMEIT)
  add_definitions(-DTIMEIT) 
endif()



if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)

        include_directories(
                test/include
        )

        # Add the test to be built
        ament_add_gtest(lane_detection_test
                test/lane_detection_test.cpp
                test/src/test_helpers.cpp
                src/lane_detection.cpp
                src/lane_detection_config.cpp
        )
        
        add_test(NAME LaneDetectionTest COMMAND lane_detection_test)

        # Link necessary libraries to the test
        target_link_libraries(lane_detection_test
                yaml-cpp
                ${OpenCV_LIBRARIES}
        )

        ament_target_dependencies(lane_detection_test ${COMMON_DEPENDENCIES})
endif()

ament_package()
