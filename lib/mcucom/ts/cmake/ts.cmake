
set(TS_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../type_print.c
    ${CMAKE_CURRENT_LIST_DIR}/../serialization_msgpack.c
    ${CMAKE_CURRENT_LIST_DIR}/../serialization_csv.c)
set(TS_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}/../..)

include(type_compiler)
