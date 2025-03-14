cmake_minimum_required(VERSION 3.8)
project(start_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(utility REQUIRED)





# Add executable
add_executable(
        start_box 
        src/start_detector.cpp 
        src/start_detector_node.cpp 
)

# Include directories
include_directories(
        include
        ${utility_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(start_box
        ${OpenCV_LIBRARIES}
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
        rclcpp
        std_msgs
        sensor_msgs
        OpenCV
        cv_bridge
        utility
)


ament_target_dependencies(start_box ${COMMON_DEPENDENCIES})

# Install headers
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
        TARGETS start_box
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/start_box
)




        



if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        
        ament_add_gtest(start_detector_test
                test/start_detector_test.cpp
                src/start_detector.cpp
        )
        

        # Link necessary libraries to the test
        target_link_libraries(start_detector_test
                ${OpenCV_LIBRARIES}
        )
        target_include_directories(start_detector_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

        ament_target_dependencies(start_detector_test ${COMMON_DEPENDENCIES})
endif()

ament_package()
