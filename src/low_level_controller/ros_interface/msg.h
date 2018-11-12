#ifndef ROS_INTERFACE_COMM_MSG_H
#define ROS_INTERFACE_COMM_MSG_H

#include "comm.h"

enum RosInterfaceCommMsgID : comm_msg_id_t {
    DEBUG_=0,
    HEARTBEAT=1,
    PING=2,
    PONG=3,
    TIME=4,
    LOG=5,
    RC_INPUT=10,
    TEST=10000,
};

#if __cplusplus < 201402L
#error "need C++14"
#endif

#include <limits>
#include <cstddef>
#include <cstdint>
#include <nop/structure.h>
#include <nop/serializer.h>
#include <nop/utility/buffer_reader.h>
#include <nop/utility/buffer_writer.h>

struct SimpleType {
    uint32_t foo;
    float bar;
    NOP_STRUCTURE(SimpleType, foo, bar);
};


#include "rc_input.hpp"
NOP_EXTERNAL_STRUCTURE(rc_input_s, nb_channels, no_signal, channel, timestamp);

#endif /* ROS_INTERFACE_COMM_MSG_H */
