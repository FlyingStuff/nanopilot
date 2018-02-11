cmake_minimum_required(VERSION 2.6)

project(msgbus-build)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")


include(cmake/msgbus.cmake)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../ts/cmake")
include(ts)

add_executable(msgbus_unittests
    ${MSGBUS_SOURCES}
    ${MSGBUS_PORT_UNITTEST_MOCK_SOURCES}
    ./tests/atomicity.cpp
    ./tests/bus_test.cpp
    ./tests/cpp_wrapper_test.cpp
    ./tests/signaling.cpp
    ./tests/subscriber_test.cpp
    ./tests/topic_test.cpp
    ${TYPE_OUTPUT_DIR}/test.c
    ./tests/runner.cpp
    )

type_compile(${CMAKE_CURRENT_SOURCE_DIR}/tests/types/test.type)
target_include_directories(msgbus_unittests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_include_directories(msgbus_unittests PUBLIC ${MSGBUS_PORT_UNITTEST_MOCK_INCLUDE})
target_include_directories(msgbus_unittests PUBLIC ${TS_TYPE_FILE_INCLUDE})
target_include_directories(msgbus_unittests PUBLIC ${TS_INCLUDE})



target_link_libraries(
    msgbus_unittests
    m
    CppUTest
    CppUTestExt
    )

enable_testing()
add_test(msgbus_unittests msgbus_unittests;-c)

list(APPEND ALL_UNIT_TESTS ${CMAKE_CURRENT_BINARY_DIR}/msgbus_unittests)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    FOREACH(t ${ALL_UNIT_TESTS})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;echo;${t})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;${t};-c)
    ENDFOREACH()
    add_custom_target(check ${ALL_UNIT_TESTS_AS_COMMAND} DEPENDS ${ALL_UNIT_TESTS})
else()
    set(ALL_UNIT_TESTS ${ALL_UNIT_TESTS} PARENT_SCOPE)
endif()



add_executable(
    msgbus_demo
    ./msgbus.c
    ./examples/posix/demo.c
    ${MSGBUS_PORT_POSIX_SOURCES}
    ${TYPE_OUTPUT_DIR}/example_type.c
    )

type_compile(${CMAKE_CURRENT_SOURCE_DIR}/examples/posix/example_type.type)
target_include_directories(msgbus_demo PUBLIC ${MSGBUS_PORT_POSIX_INCLUDE})
target_include_directories(msgbus_demo PUBLIC ${TS_TYPE_FILE_INCLUDE})
target_include_directories(msgbus_demo PUBLIC ${TS_INCLUDE})

target_link_libraries(
    msgbus_demo
    pthread
    )

