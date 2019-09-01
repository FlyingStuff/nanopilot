cmake_minimum_required(VERSION 3.5)
project(attitude_ekf)

set(CMAKE_CXX_STANDARD 14)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) #  -Wpedantic
endif()

add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autopilot_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(attitude_ekf
    src/attitude_ekf.cpp
)
target_include_directories(attitude_ekf PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(attitude_ekf rclcpp std_msgs sensor_msgs geometry_msgs autopilot_msgs tf2 tf2_ros tf2_sensor_msgs)

install(TARGETS attitude_ekf
  DESTINATION lib/${PROJECT_NAME})



add_executable(attitude_pos_ekf
    src/attitude_pos_ekf.cpp
)
target_include_directories(attitude_pos_ekf PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(attitude_pos_ekf rclcpp std_msgs sensor_msgs geometry_msgs autopilot_msgs tf2 tf2_ros tf2_sensor_msgs)

install(TARGETS attitude_pos_ekf
  DESTINATION lib/${PROJECT_NAME})

ament_package()
