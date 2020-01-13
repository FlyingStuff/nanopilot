cmake_minimum_required(VERSION 3.5)
project(pid)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) #  -Wpedantic
endif()

find_package(ament_cmake REQUIRED)

add_library(pid
    src/pid.cpp
)
target_include_directories(pid PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_target_dependencies(pid)

install(TARGETS pid
  DESTINATION lib)

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)

