cmake_minimum_required(VERSION 3.5)
project(llc_ros_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(llc_ros_interface
    llc_ros_interface.cpp
    ../low_level_controller/llc_ros_interface/comm.cpp
    ../low_level_controller/lib/comm/serial-datagram/serial_datagram.c
    ../low_level_controller/lib/comm/serial-datagram/serial_datagram_buffer_writer.c
    ../low_level_controller/lib/comm/crc/crc32.c
)
target_include_directories(llc_ros_interface PUBLIC
    ../low_level_controller/lib/comm/
    ../low_level_controller/llc_ros_interface/
)

ament_target_dependencies(llc_ros_interface rclcpp std_msgs)

install(TARGETS llc_ros_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()
