
set(NET_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../net.c)
set(NET_PORT_POSIX_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../../port/posix/mcucom_port_sync.c)
set(NET_PORT_UNITTEST_MOCK_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../../port/unittest_mock/mcucom_port_assert.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../../port/unittest_mock/mcucom_port_sync.cpp)


set(NET_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}/../..)
set(NET_PORT_POSIX_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}/../../port/posix)
set(NET_PORT_UNITTEST_MOCK_INCLUDE
    ${CMAKE_CURRENT_LIST_DIR}/../../port/unittest_mock)
