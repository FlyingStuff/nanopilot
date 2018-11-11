cmake_minimum_required(VERSION 2.6)

project(low-level-controller-unittests)
set(CMAKE_BUILD_TYPE Debug)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")

include(cmp)
include(cmp_mem_access)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -g")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")

add_executable(
    low-level-controller-unittests
    ../../src/tests/runner.cpp
    ${CMP_SOURCES}
    ../../lib/comm/cmp_mem_access/cmp_mem_access.c
    ../../lib/comm/crc/crc32.c
    ../../src/hott/sumd.c
    ../../src/hott/telemetry.c
    ../../src/tests/hott/telemetry_test.cpp
    ../../lib/parameter/parameter.c
    ../../lib/parameter/parameter_msgpack.c
    ../../lib/parameter/parameter_print.c
    ../../lib/parameter/tests/msgpack_test.cpp
    ../../lib/parameter/tests/parameter_print_test.cpp
    ../../lib/parameter/tests/parameter_test.cpp
    ../../lib/parameter/tests/parameter_types_test.cpp
    ../../lib/comm/serial-datagram/serial_datagram.c
    ../../lib/comm/serial-datagram/serial_datagram_buffer_writer.c
    ../../lib/comm/cmp_mem_access/cmp_mem_access_test.cpp
    ../../lib/comm/crc/test/crc32_test.cpp
    ../../lib/comm/serial-datagram/tests/serial_datagram_buffer_writer_tests.cpp
    ../../lib/comm/serial-datagram/tests/serial_datagram_test.cpp
    ../../src/timestamp.c
    ../../src/tests/timestamp_mock.c
    ../../src/tests/timestamp_mock_test.cpp
    ../../src/tests/timestamp_test.cpp
    ../../src/log.c
    ../../src/tests/log_test.cpp
    ../../src/pid.c
    ../../src/tests/pid_test.cpp
    )



target_include_directories(low-level-controller-unittests PUBLIC ../../lib/)
target_include_directories(low-level-controller-unittests PUBLIC ../../lib/comm/)
target_include_directories(low-level-controller-unittests PUBLIC ../../src/eigen/)
target_include_directories(low-level-controller-unittests PUBLIC ../../src/)
target_include_directories(low-level-controller-unittests PUBLIC ../../src/tests/mock)

target_link_libraries(
    low-level-controller-unittests
    m
    CppUTest
    CppUTestExt
    )

enable_testing()
add_test(low-level-controller-unittests low-level-controller-unittests;-c)

