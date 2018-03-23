cmake_minimum_required(VERSION 2.6)

project(unittest-build)
set(CMAKE_BUILD_TYPE Debug)


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/mcucom/msgbus/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/mcucom/ts/cmake")

include(msgbus)
include(ts)
include(cmp)
include(cmp_mem_access)
include(fatfs)



SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -g")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")


# add unit test targets (from other CMakeLists.txt)
add_subdirectory(lib/mcucom/msgbus)
add_subdirectory(lib/mcucom/ts)
add_subdirectory(lib/mcucom/net)
add_subdirectory(lib/drivers/sensors)
# ...


add_executable(
    ins-board-unittests
    ./src/tests/runner.cpp
    ${FATFS_SOURCES}
    ./src/tests/mock/fatfs_diskio_mem_drv.c
    ./src/tests/fatfs_tests.cpp
    ./src/sdlog.c
    ./src/tests/sdlog_test.cpp
    ./src/sdcard.c
    ./src/tests/sdcard_test.cpp
    ${CMP_SOURCES}
    ./lib/comm/cmp_mem_access/cmp_mem_access.c
    ./lib/comm/crc/crc32.c
    ./lib/comm/datagram-messages/msg_dispatcher.c
    ./lib/comm/datagram-messages/service_call.c
    ./src/hott/sumd.c
    ./src/hott/telemetry.c
    ./src/tests/hott/telemetry_test.cpp
    ./src/msgbus_scheduler.c
    ./src/tests/msgbus_scheduler_test.cpp
    ${MSGBUS_SOURCES}
    ${MSGBUS_PORT_UNITTEST_MOCK_SOURCES}
    ${TS_SOURCES}
    ${TYPE_OUTPUT_DIR}/test.c
    ./src/parameter/parameter.c
    ./src/parameter/parameter_msgpack.c
    ./src/parameter/parameter_print.c
    ./src/rate_limiter.c
    ./lib/comm/serial-datagram/serial_datagram.c
    ./lib/comm/serial-datagram/serial_datagram_buffer_writer.c
    ./src/timestamp.c
    ./lib/comm/cmp_mem_access/cmp_mem_access_test.cpp
    ./lib/comm/crc/test/crc32_test.cpp
    ./lib/comm/datagram-messages/tests/msg_dispatcher_test.cpp
    ./lib/comm/datagram-messages/tests/service_call_test.cpp
    ./src/parameter/tests/msgpack_test.cpp
    ./src/parameter/tests/parameter_print_test.cpp
    ./src/parameter/tests/parameter_test.cpp
    ./src/parameter/tests/parameter_types_test.cpp
    ./lib/comm/serial-datagram/tests/serial_datagram_buffer_writer_tests.cpp
    ./lib/comm/serial-datagram/tests/serial_datagram_test.cpp
    ./src/tests/rate_limiter_test.cpp
    ./src/tests/timestamp_mock.c
    ./src/tests/timestamp_mock_test.cpp
    ./src/tests/timestamp_test.cpp
    ./src/log.c
    ./src/tests/log_test.cpp
    )


type_compile(${CMAKE_CURRENT_SOURCE_DIR}/src/tests/types/test.type)
target_include_directories(ins-board-unittests PUBLIC ${TS_INCLUDE})
target_include_directories(ins-board-unittests PUBLIC ${TS_TYPE_FILE_INCLUDE})
target_include_directories(ins-board-unittests PUBLIC ${MSGBUS_INCLUDE})
target_include_directories(ins-board-unittests PUBLIC ${MSGBUS_PORT_UNITTEST_MOCK_INCLUDE})
target_include_directories(ins-board-unittests PUBLIC ${FATFS_INCLUDE})

target_include_directories(ins-board-unittests PUBLIC ./lib/comm/)
target_include_directories(ins-board-unittests PUBLIC ./src/eigen/)
target_include_directories(ins-board-unittests PUBLIC ./src/flight-stack/src/)
target_include_directories(ins-board-unittests PUBLIC ./src/)
target_include_directories(ins-board-unittests PUBLIC ./src/tests/mock)

target_link_libraries(
    ins-board-unittests
    m
    CppUTest
    CppUTestExt
    )

enable_testing()
add_test(ins-board-unittests ins-board-unittests;-c)

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

