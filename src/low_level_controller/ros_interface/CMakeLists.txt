cmake_minimum_required(VERSION 3.5)
project(llc_ros_interface)

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


add_executable(llc_ros_interface
    llc_ros_interface.cpp
    comm.cpp
    ../lib/comm/serial-datagram/serial_datagram.c
    ../lib/comm/serial-datagram/serial_datagram_buffer_writer.c
    ../lib/comm/crc/crc32.c
)
target_include_directories(llc_ros_interface PUBLIC
    ../lib/comm/
    ../lib/libnop/include/
    ../src/
)

ament_target_dependencies(llc_ros_interface rclcpp std_msgs sensor_msgs geometry_msgs autopilot_msgs)

install(TARGETS llc_ros_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()
