cmake_minimum_required(VERSION 2.6)

project(flight-stack)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

include_directories(src)
include_directories(lib)
include_directories(lib/eigen)
include_directories(lib/cppzmq)

file(GLOB_RECURSE src src/*.cpp)

add_executable(
    tests
    ${src}
    )

target_link_libraries(
    tests
    CppUTest
    CppUTestExt
    )


