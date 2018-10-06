#ifndef ROS_INTERFACE_COMM_MSG_H
#define ROS_INTERFACE_COMM_MSG_H

#include "comm.h"

enum RosInterfaceCommMsgID : comm_msg_id_t {
    HEARTBEAT=1,
    PING=2,
    PONG=3,
    TIME=4,
};


#endif /* ROS_INTERFACE_COMM_MSG_H */
