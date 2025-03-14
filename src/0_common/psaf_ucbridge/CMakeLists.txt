cmake_minimum_required(VERSION 3.16)
project(psaf_ucbridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

#######################
# Libserial
#######################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Prefer -pthread compiler and linker flag when using libpthread. This must
# be set before call to find_package(Threads).
#
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)
#
# If you've installed LibSerial in a non-standard location, please add it
# to PKG_CONFIG_PATH first. For example, if LibSerial is installed under
# /opt/libserial, set PKG_CONFIG_PATH environment variable to
# /opt/libserial/lib/pkgconfig/ before running cmake for this example.
#
# export PKG_CONFIG_PATH=/opt/libserial/lib/pkgconfig/
#
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)
#######################

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(psaf_ucbridge_msgs REQUIRED)
#find_package(libserial REQUIRED)

# Header files
include_directories(include ${rclcpp_INCLUDE_DIRS})

# UC Bridge Library
add_library(uc_bridge_lib
        src/communication/board_communication.cpp
        src/uc_node.cpp
        src/communication/command.cpp
        src/communication/request.cpp
        src/communication/communication.cpp
        src/communication/reading_thread.cpp
        src/communication/data_receiver.cpp
#        src/communication/data_logger.cpp
        src/configuration/configuration.cpp
        src/configuration/static_config.cpp
        src/configuration/dynamic_config.cpp
        src/service_functions/service_functions.cpp
        src/utils.cpp
        src/sensor_groups/sensor_group.cpp
        src/sensor_groups/display_txt.cpp
        src/sensor_groups/channel_description.cpp
        src/logging/logger.cpp
        src/logging/logger_factory.cpp src/communication/raw_message.cpp)

target_precompile_headers(uc_bridge_lib
        PRIVATE <string>
        <sstream>
        <iostream>
        <fstream>
        <memory>
        <functional>
        <chrono>
        <algorithm>
        <stdexcept>
        <future>
        <thread>
        <mutex>
        <condition_variable>
        <map>
        <vector>
        <deque>
        [["rclcpp/rclcpp.hpp"]]
        [["std_msgs/msg/int16.hpp"]]
        [["std_msgs/msg/u_int8.hpp"]]
        [["std_msgs/msg/float32.hpp"]]
        [["sensor_msgs/msg/imu.hpp"]]
        [["sensor_msgs/msg/range.hpp"]]
        [["sensor_msgs/msg/magnetic_field.hpp"]])

target_link_libraries(uc_bridge_lib ${rclcpp_LIBRARIES} ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})

# UC Bridge
add_executable(uc_bridge src/uc_bridge.cpp)
target_precompile_headers(uc_bridge REUSE_FROM uc_bridge_lib)
target_link_libraries(uc_bridge uc_bridge_lib)

# UC Board Mock
add_executable(uc_board_mock test/uc_board_mock.cpp include/psaf_ucbridge/communication/raw_message.hpp)
target_link_libraries(uc_board_mock ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})

# ROS2 dependencies
ament_target_dependencies(uc_bridge rclcpp std_msgs sensor_msgs psaf_ucbridge_msgs)
ament_target_dependencies(uc_bridge_lib rclcpp std_msgs sensor_msgs psaf_ucbridge_msgs)


# Install
install(TARGETS
        uc_bridge
        uc_board_mock
        DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    # set(ament_cmake_lint_cmake_FOUND TRUE)
    # set(ament_cmake_uncrustify_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    # GTest
    find_package(ament_cmake_gtest)

    # GMock
    find_package(ament_cmake_gmock)

    ament_add_gtest(command_test test/command_test.cpp)
    ament_add_gmock(channel_test test/channel_test.cpp)
    ament_add_gmock(sensor_group_test test/sensor_group_test.cpp)
    ament_add_gmock(message_queue_test test/message_queue_test.cpp)
    ament_add_gmock(uc_bridge_thread_test test/uc_bridge_thread_test.cpp)
    ament_add_gtest(request_test test/request_test.cpp)
    #ament_add_gtest(response_test test/response_test.cpp ${SOURCE_FILES})
    ament_add_gtest(utils_test test/utils_test.cpp)
    ament_add_gmock(logger_test test/logger_test.cpp)
    ament_add_gmock(config_test test/config_test.cpp)
    ament_add_gmock(service_functions_test test/service_functions_test.cpp)
#    ament_add_gmock(communication_test test/communication_test.cpp)


    target_link_libraries(command_test uc_bridge_lib)
    target_link_libraries(channel_test uc_bridge_lib)
    target_link_libraries(sensor_group_test uc_bridge_lib)
    target_link_libraries(message_queue_test uc_bridge_lib)
    target_link_libraries(uc_bridge_thread_test uc_bridge_lib)
    target_link_libraries(request_test uc_bridge_lib)
    #target_link_libraries(response_test ${rclcpp_LIBRARIES} ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})
    target_link_libraries(utils_test uc_bridge_lib)
    target_link_libraries(logger_test uc_bridge_lib)
    target_link_libraries(config_test uc_bridge_lib)
    target_link_libraries(service_functions_test uc_bridge_lib)
#    target_link_libraries(communication_test uc_bridge_lib)
endif()

ament_package()
