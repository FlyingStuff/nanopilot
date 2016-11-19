cmake_minimum_required(VERSION 2.6)

project(unittest-build)
set(CMAKE_BUILD_TYPE Debug)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/src/msgbus")
find_package(msgbus)

add_subdirectory(src/msgbus) # to build tests

include(${CMAKE_CURRENT_LIST_DIR}/src/msgbus/cmake/type_compiler.cmake)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")


add_executable(
    ins-board-unittests
    ./src/tests/runner.cpp
    ./src/cmp/cmp.c
    ./src/cmp_mem_access/cmp_mem_access.c
    ./src/crc/crc32.c
    ./src/datagram-messages/msg_dispatcher.c
    ./src/datagram-messages/service_call.c
    ./src/hott/sumd.c
    ./src/msgbus_scheduler.c
    ./src/tests/msgbus_scheduler_test.cpp
    ${MSGBUS_SOURCES}
    ${MSGBUS_PORT_UNITTEST_MOCK_SOURCES}
    ${CMAKE_CURRENT_BINARY_DIR}/types/test.c
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


type_compile(${CMAKE_CURRENT_SOURCE_DIR}/src/tests/types/test.type)
target_include_directories(ins-board-unittests PUBLIC ${TYPE_FILE_INCLUDE_DIR})
target_include_directories(ins-board-unittests PUBLIC ${MSGBUS_INCLUDE_DIR})
target_include_directories(ins-board-unittests PUBLIC ${MSGBUS_PORT_UNITTEST_MOCK_INCLUDE_DIR})

target_include_directories(ins-board-unittests PUBLIC ./src/eigen/)
target_include_directories(ins-board-unittests PUBLIC ./src/flight-stack/src/)
target_include_directories(ins-board-unittests PUBLIC ./src/)

target_link_libraries(
    ins-board-unittests
    m
    CppUTest
    CppUTestExt
    )

enable_testing()
add_test(ins-board-unittests ins-board-unittests)

list(APPEND ALL_UNIT_TESTS ${CMAKE_CURRENT_BINARY_DIR}/ins-board-unittests)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    FOREACH(t ${ALL_UNIT_TESTS})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;echo;${t})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;${t};-c)
    ENDFOREACH()
    add_custom_target(check ${ALL_UNIT_TESTS_AS_COMMAND} DEPENDS ${ALL_UNIT_TESTS})
else()
    set(ALL_UNIT_TESTS ${ALL_UNIT_TESTS} PARENT_SCOPE)
endif()

