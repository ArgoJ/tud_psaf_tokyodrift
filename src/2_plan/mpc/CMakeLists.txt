cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS Abhängigkeiten ---
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(utility REQUIRED)
find_package(helpers REQUIRED)
find_package(timer REQUIRED)

# --- ACADOS ---
set(ACADOS_SOURCE_DIR_VAR "$ENV{ACADOS_SOURCE_DIR}")
set(ACADOS_INCLUDE_PATH ${ACADOS_SOURCE_DIR_VAR}/include)
set(ACADOS_LIB_DIR ${ACADOS_SOURCE_DIR_VAR}/lib)
if(NOT EXISTS ${ACADOS_LIB_DIR})
    message(FATAL_ERROR "Acados lib directory not found: ${ACADOS_LIB_DIR}")
endif()

set(ACADOS_GENERATED_CODE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code)
# if(NOT EXISTS ${ACADOS_GENERATED_CODE_DIR})
#     message(FATAL_ERROR "Acados generated code directory not found: ${ACADOS_GENERATED_CODE_DIR}")
# endif()

message(STATUS "Using Acados generated code from: ${ACADOS_GENERATED_CODE_DIR}")

# Definiere den Pfad zu deinem Python Skript
set(ACADOS_PYTHON_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/acados_mpc.py)
message(STATUS "Using ACADOS Python script: ${ACADOS_PYTHON_SCRIPT}")

# --- Executable ---
add_executable(mpc 
    src/mpc.cpp
)

# --- Generate ACADOS code ---
set(ACADOS_VENV_DIR $ENV{HOME}/envs/ac_env)

# Definiere den Pfad zum Python-Interpreter innerhalb des venv
set(VENV_PYTHON_EXECUTABLE ${ACADOS_VENV_DIR}/bin/python3)

# Überprüfe, ob der venv-Interpreter existiert
if(NOT EXISTS ${VENV_PYTHON_EXECUTABLE})
    message(FATAL_ERROR "Python interpreter for ACADOS venv not found: ${VENV_PYTHON_EXECUTABLE}. Please check your ACADOS_VENV_DIR.")
endif()

# Setze den Python-Interpreter für das Skript
add_custom_target(generate_acados_code
    COMMAND ${VENV_PYTHON_EXECUTABLE} ${ACADOS_PYTHON_SCRIPT}
    # WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts # Optional: Setzt das Arbeitsverzeichnis für das Skript
    COMMENT "Executing acados_mpc.py to generate ACADOS code..."
)
add_dependencies(mpc generate_acados_code)

# --- Include directories ---
include_directories(
    include
    ${utility_INCLUDE_DIRS}
    ${ACADOS_GENERATED_CODE_DIR}
    ${ACADOS_INCLUDE_PATH}
    ${ACADOS_INCLUDE_PATH}/acados
    ${ACADOS_INCLUDE_PATH}/blasfeo/include
    ${ACADOS_INCLUDE_PATH}/hpipm/include
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

# Install generated ACADOS C-Code-Files
file(GLOB_RECURSE ACADOS_GENERATED_SO_LIBS
    "${ACADOS_GENERATED_CODE_DIR}/*.so"
)

# Stelle sicher, dass wir nur die tatsächlichen Dateien installieren
if(ACADOS_GENERATED_SO_LIBS)
    install(
        FILES ${ACADOS_GENERATED_SO_LIBS}
        DESTINATION lib
    )
    message(STATUS "Installed ACADOS generated shared libraries (recursively found): ${ACADOS_GENERATED_SO_LIBS}")
else()
    message(WARNING "No .so files found in ${ACADOS_GENERATED_CODE_DIR} (recursively) to install. Is the Python script generating them correctly?")
endif()

# OPTIONS
option(TIMEIT "Enable timeit features" ON)

# Define TIMEIT macro for timing purposes
if(TIMEIT)
  add_definitions(-DTIMEIT) 
endif()


# --- Finalization ---
ament_package()