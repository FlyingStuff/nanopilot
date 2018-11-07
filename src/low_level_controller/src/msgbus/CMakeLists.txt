cmake_minimum_required(VERSION 2.6)

project(msgbus-unittests)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")


add_executable(msgbus_unittests
    ./msgbus_unittests.cpp
    )


target_link_libraries(
    msgbus_unittests
    m
    CppUTest
    CppUTestExt
    )

enable_testing()
add_test(msgbus_unittests msgbus_unittests;-c)


add_executable(example ./example.cpp)
add_executable(example2 ./example2.cpp)

