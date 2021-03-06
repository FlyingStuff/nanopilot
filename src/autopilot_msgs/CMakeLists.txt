cmake_minimum_required(VERSION 3.5)

project(autopilot_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(autopilot_msgs
  "msg/RCInput.msg"
  "msg/ActuatorPositions.msg"
  "msg/ActuatorPositionsStamped.msg"
  "msg/AttitudeControllerStatus.msg"
  "msg/AttitudeTrajectorySetpoint.msg"
  "msg/ControlStatus.msg"
  "msg/PositionTrajectorySetpoint.msg"
  "msg/AccelerationTrajectorySetpoint.msg"
  "msg/TimeSyncStat.msg"
  "srv/SendMsgpackConfig.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_package()