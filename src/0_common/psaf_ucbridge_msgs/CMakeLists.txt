cmake_minimum_required(VERSION 3.5)
project(psaf_ucbridge_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Led.msg"
        "msg/Daq.msg"
        "msg/UnkDaq.msg"
        "msg/EncCm.msg"
        "msg/EncStep.msg"
        "msg/RawMessage.msg"
        "msg/Pbs.msg"
        "msg/Vbat.msg"
        "msg/Display.msg"
        "srv/ReqVer.srv"
        "srv/ReqId.srv"
        "srv/ReqSid.srv"
        "srv/ReqSteer.srv"
        "srv/ReqDrv.srv"
        "srv/ReqDms.srv"
        "srv/ReqChs.srv"
        "srv/ReqGet.srv"
        "srv/ReqGrp.srv"
        "srv/ReqVout.srv"
        "srv/ReqUs.srv"
        "srv/ReqImu.srv"
        "srv/ReqMag.srv"
        "srv/ReqEnc.srv"
        "srv/ReqRaw.srv"
        DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
