cmake_minimum_required(VERSION 2.6)

project(unittest-build)
set(CMAKE_BUILD_TYPE Debug)


include_directories(./src/eigen/)
include_directories(./src/flight-stack/src/)
include_directories(./src/)
include_directories(./src/msgbus/tests)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

add_executable(
    tests
    ./src/tests/runner.cpp
    ./src/cmp/cmp.c
    ./src/cmp_mem_access/cmp_mem_access.c
    ./src/crc/crc32.c
    ./src/datagram-messages/msg_dispatcher.c
    ./src/datagram-messages/service_call.c
    ./src/hott/sumd.c
    ./src/msgbus/msgbus.c
    ./src/msgbus/serialization_msgpack.c
    ./src/msgbus/type_print.c
    ./src/parameter/parameter.c
    ./src/parameter/parameter_msgpack.c
    ./src/parameter/parameter_print.c
    ./src/rate_limiter.c
    ./src/rcservo/rcservo.c
    ./src/serial-datagram/serial_datagram.c
    ./src/serial-datagram/serial_datagram_buffer_writer.c
    ./src/timestamp/timestamp.c
    ./src/cmp_mem_access/cmp_mem_access_test.cpp
    ./src/crc/test/crc32_test.cpp
    ./src/datagram-messages/tests/msg_dispatcher_test.cpp
    ./src/datagram-messages/tests/service_call_test.cpp
    ./src/msgbus/tests/atomicity.cpp
    ./src/msgbus/tests/bus_test.cpp
    ./src/msgbus/tests/cpp_wrapper_test.cpp
    ./src/msgbus/tests/mocks/assert.cpp
    ./src/msgbus/tests/mocks/synchronization.cpp
    ./src/msgbus/tests/serialization_msgpack_test.cpp
    ./src/msgbus/tests/signaling.cpp
    ./src/msgbus/tests/subscriber_test.cpp
    ./src/msgbus/tests/topic_test.cpp
    ./src/msgbus/tests/type_print_test.cpp
    ./src/msgbus/tests/types/test.c
    ./src/parameter/tests/msgpack_test.cpp
    ./src/parameter/tests/parameter_print_test.cpp
    ./src/parameter/tests/parameter_test.cpp
    ./src/parameter/tests/parameter_types_test.cpp
    ./src/rcservo/rcservo_test.cpp
    ./src/serial-datagram/tests/serial_datagram_buffer_writer_tests.cpp
    ./src/serial-datagram/tests/serial_datagram_test.cpp
    ./src/tests/rate_limiter_test.cpp
    ./src/tests/timestamp_mock.c
    ./src/tests/timestamp_mock_test.cpp
    ./src/timestamp/timestamp_test.cpp
    )


target_link_libraries(
    tests
    m
    CppUTest
    CppUTestExt
    )

# Run unit tests
add_custom_target(check ./tests -c DEPENDS tests)


