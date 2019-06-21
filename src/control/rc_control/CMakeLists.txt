cmake_minimum_required(VERSION 3.5)
project(rc_control)

set(CMAKE_CXX_STANDARD 14)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) #  -Wpedantic
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autopilot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(rc_control
    src/rc_ctrl.cpp
)
target_include_directories(rc_control PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(rc_control rclcpp std_msgs sensor_msgs geometry_msgs autopilot_msgs)

install(TARGETS rc_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
