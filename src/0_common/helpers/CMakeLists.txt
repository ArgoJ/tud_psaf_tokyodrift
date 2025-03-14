cmake_minimum_required(VERSION 3.8)
project(helpers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Declare header-only library
add_library(helpers INTERFACE)
target_include_directories(helpers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Link dependencies to the interface target
target_link_libraries(helpers INTERFACE
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    Eigen3::Eigen
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    Eigen3
)

ament_export_dependencies(${COMMON_DEPENDENCIES})
ament_export_include_directories(include)

# Install header files
install(
    DIRECTORY include/
    DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(helper_test 
        test/filter_test.cpp
        test/integrator_test.cpp
        test/point_msg_helper_test.cpp
    )

    target_include_directories(helper_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(helper_test ${COMMON_DEPENDENCIES})
    
endif()

ament_package()
