cmake_minimum_required(VERSION 2.6)

project(net-tests)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(net)


add_executable(
    net_unittests
    ${NET_SOURCES}
    ${NET_PORT_UNITTEST_MOCK_SOURCES}
    ./tests/test_net.cpp
    ./tests/test_locking.cpp
    ./tests/runner.cpp
    )

target_include_directories(net_unittests PUBLIC ${NET_PORT_UNITTEST_MOCK_INCLUDE})

target_link_libraries(
    net_unittests
    m
    CppUTest
    CppUTestExt
    )

enable_testing()
add_test(net_unittests net_unittests;-c)

list(APPEND ALL_UNIT_TESTS ${CMAKE_CURRENT_BINARY_DIR}/net_unittests)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    FOREACH(t ${ALL_UNIT_TESTS})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;echo;${t})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;${t};-c)
    ENDFOREACH()
    add_custom_target(check ${ALL_UNIT_TESTS_AS_COMMAND} DEPENDS ${ALL_UNIT_TESTS})
else()
    set(ALL_UNIT_TESTS ${ALL_UNIT_TESTS} PARENT_SCOPE)
endif()

