#ifndef ROS_COMM_HPP
#define ROS_COMM_HPP

#include <ros_interface/comm.h>

extern comm_interface_t comm_if;

void ros_comm_init(BaseSequentialStream *sd);

#endif /* ROS_COMM_HPP */
