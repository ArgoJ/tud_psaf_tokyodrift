cmake_minimum_required(VERSION 3.8)
project(uc_com_simple)

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
find_package(timer REQUIRED)


add_executable(
        uc_com_simple
        src/uc_com_node.cpp     
)

include_directories(
        include
)

ament_target_dependencies(uc_com_simple 
        rclcpp
        utility
        geometry_msgs
        sensor_msgs
        timer
)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS uc_com
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/uc_com
)

# OPTIONS
option(TIMEIT "Enable timeit features" ON)

# Define TIMEIT macro for timing purposes
if(TIMEIT)
  add_definitions(-DTIMEIT) 
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
endif()

ament_package()
