cmake_minimum_required(VERSION 2.6)

project(msgbus-build)
set(CMAKE_BUILD_TYPE Debug)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(ts)

include(cmp OPTIONAL RESULT_VARIABLE CMP_FOUND)
if (NOT CMP_FOUND) # download if not found
    set(CMP_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/cmp/cmp.c)
    set(CMP_INCLUDE ${CMAKE_CURRENT_BINARY_DIR})
    execute_process(COMMAND git clone https://github.com/camgunz/cmp.git -b v16
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif (NOT CMP_FOUND)

include(cmp_mem_access OPTIONAL RESULT_VARIABLE CMP_MEM_ACCESS_FOUND)
if (NOT CMP_FOUND) # download if not found
    set(CMP_MEM_ACCESS_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/cmp_mem_access/cmp_mem_access.c)
    set(CMP_MEM_ACCESS_INCLUDE ${CMAKE_CURRENT_BINARY_DIR})
    execute_process(COMMAND git clone https://github.com/Stapelzeiger/cmp_mem_access.git
                    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif (NOT CMP_FOUND)


add_executable(
    ts_unittests
    ${TS_SOURCES}
    ./tests/serialization_msgpack_test.cpp
    ./tests/serialization_csv_test.cpp
    ./tests/type_print_test.cpp
    ${TYPE_OUTPUT_DIR}/test.c
    ./tests/runner.cpp
    ${CMP_SOURCES}
    ${CMP_MEM_ACCESS_SOURCES}
    )

type_compile(${CMAKE_CURRENT_SOURCE_DIR}/tests/types/test.type)
target_include_directories(ts_unittests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_include_directories(ts_unittests PUBLIC ${TS_INCLUDE})
target_include_directories(ts_unittests PUBLIC ${TS_TYPE_FILE_INCLUDE})
target_include_directories(ts_unittests PUBLIC ${CMP_INCLUDE})
target_include_directories(ts_unittests PUBLIC ${CMP_MEM_ACCESS_INCLUDE})


target_link_libraries(
    ts_unittests
    m
    CppUTest
    CppUTestExt
    )

enable_testing()
add_test(ts_unittests ts_unittests;-c)

list(APPEND ALL_UNIT_TESTS ${CMAKE_CURRENT_BINARY_DIR}/ts_unittests)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    FOREACH(t ${ALL_UNIT_TESTS})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;echo;${t})
        LIST(APPEND ALL_UNIT_TESTS_AS_COMMAND COMMAND;${t};-c)
    ENDFOREACH()
    add_custom_target(check ${ALL_UNIT_TESTS_AS_COMMAND} DEPENDS ${ALL_UNIT_TESTS})
else()
    set(ALL_UNIT_TESTS ${ALL_UNIT_TESTS} PARENT_SCOPE)
endif()

