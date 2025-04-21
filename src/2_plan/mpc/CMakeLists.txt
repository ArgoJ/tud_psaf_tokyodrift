cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS Abhängigkeiten ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(utility REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)

# --- ACADOS ---
set(ACADOS_LIB_DIR /home/argo/acados_ws/acados/lib)
if(NOT EXISTS ${ACADOS_LIB_DIR})
    message(FATAL_ERROR "Acados lib directory not found: ${ACADOS_LIB_DIR}")
endif()

set(ACADOS_GENERATED_CODE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code)
if(NOT EXISTS ${ACADOS_GENERATED_CODE_DIR})
    message(FATAL_ERROR "Acados generated code directory not found: ${ACADOS_GENERATED_CODE_DIR}")
endif()

message(STATUS "Using Acados generated code from: ${ACADOS_GENERATED_CODE_DIR}")


# --- Executable ---
add_executable(mpc 
    src/mpc.cpp
)

# --- Include directories ---
include_directories(
    include
    ${utility_INCLUDE_DIRS}
)

# Include Directories
target_include_directories(mpc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Link Libraries
target_link_libraries(mpc
    ${ACADOS_GENERATED_CODE_DIR}/libacados_ocp_solver_bicycle_model.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    m
)

# Declare dependencies
set(COMMON_DEPENDENCIES 
    rclcpp
    geometry_msgs
    visualization_msgs
    utility
    helpers
    timer
)

ament_target_dependencies(mpc ${COMMON_DEPENDENCIES})

# --- Installation ---
install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS mpc
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# OPTIONS
option(TIMEIT "Enable timeit features" ON)

# Define TIMEIT macro for timing purposes
if(TIMEIT)
  add_definitions(-DTIMEIT) 
endif()


# --- Finalization ---
ament_package()