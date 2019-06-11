#ifndef ROS_INTERFACE_COMM_MSG_H
#define ROS_INTERFACE_COMM_MSG_H


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

#include "comm.h"

enum RosInterfaceCommMsgID : comm_msg_id_t {
    DEBUG_=0,
    HEARTBEAT=1,
    PING=2,
    PONG=3,
    TIME=4,
    LOG=5,
    RC_INPUT=10,
    ACTUATOR_OUTPUT=11,
    RATE_CTRL_SETPOINT_RPY=12,
    RATE_CTRL_MEASURED_RPY=13,
    RATE_CTRL_OUTPUT_RPY=14,
    IMU=20,
    // ACCELEROMETER=21,
    MAGNETOMETER=22,
    BAROMETER=23,
    SET_PARAMETERS=50,
    SET_PARAMETERS_RES=51,
    SAVE_PARAMETERS=52,
    SAVE_PARAMETERS_RES=53,
    GET_PARAMETER=54,
    GET_PARAMETER_RES=55,
};


#include "rc_input.hpp"
NOP_EXTERNAL_STRUCTURE(rc_input_s, nb_channels, no_signal, channel, timestamp);
#include "sensors.hpp"
NOP_EXTERNAL_STRUCTURE(quaternion_s, w, x, y, z);
NOP_EXTERNAL_STRUCTURE(rate_gyro_sample_t, rate, timestamp, accumulated_angle);
NOP_EXTERNAL_STRUCTURE(accelerometer_sample_t, acceleration, timestamp);
NOP_EXTERNAL_STRUCTURE(barometer_sample_t, pressure, temperature, timestamp);
NOP_EXTERNAL_STRUCTURE(magnetometer_sample_t, magnetic_field, timestamp);

#endif /* ROS_INTERFACE_COMM_MSG_H */
