cmake_minimum_required(VERSION 3.8)
project(utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(utility
  "msg/Control.msg"
  "msg/ReferenceLine.msg"
  "msg/Startbox.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/VehicleState.msg"
  "msg/FilteredHall.msg"
  "msg/FilteredIMU.msg"
  "msg/FusedSensor.msg"
  "msg/ObstacleDetection.msg"
  DEPENDENCIES
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
)

install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()