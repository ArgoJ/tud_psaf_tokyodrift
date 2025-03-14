cmake_minimum_required(VERSION 3.8)
project(simulated_control)

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
find_package(utility REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)


# Add executable
add_executable(
    simulated_control 
    src/simulated_control_node.cpp 
    src/simulated_control.cpp 
    src/simulated_control_config.cpp
)

# Include directories
include_directories(
    include
    ${utility_INCLUDE_DIRS}
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    utility
    helpers
    timer
)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

ament_target_dependencies(simulated_control ${COMMON_DEPENDENCIES})

# Install headers
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
    TARGETS simulated_control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/simulated_control
)

# OPTIONS
option(TIMEIT "Enable timeit features" ON)

# Define TIMEIT macro for timing purposes
if(TIMEIT)
  add_definitions(-DTIMEIT) 
endif()


if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    include_directories(
        test/include
    )

    ament_add_gtest(simulated_control_test
        test/simulated_control_test.cpp
        src/simulated_control.cpp
        src/simulated_control_config.cpp
    )

    add_test(NAME SimulatedControlTest COMMAND simulated_control_test)

    ament_target_dependencies(simulated_control_test ${COMMON_DEPENDENCIES})
endif()

ament_package()

