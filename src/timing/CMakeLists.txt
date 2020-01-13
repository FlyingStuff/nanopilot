cmake_minimum_required(VERSION 3.5)
project(timing)

set(CMAKE_CXX_STANDARD 14)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) #  -Wpedantic
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(pub
    src/pub.cpp
)
ament_target_dependencies(pub rclcpp std_msgs geometry_msgs)
install(TARGETS pub
  DESTINATION lib/${PROJECT_NAME})


add_executable(sub
    src/sub.cpp
)
ament_target_dependencies(sub rclcpp std_msgs geometry_msgs)
install(TARGETS sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
