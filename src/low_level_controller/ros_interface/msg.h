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
    TIME_SYNC=6,
    RC_INPUT=10,
    ACTUATOR_OUTPUT=11,
    CONTROL_STATUS=12,
    AP_LATENCY=13,
    IMU=20,
    MAGNETOMETER=22,
    BAROMETER=23,
    ATTITUDE_FILTER_OUTPUT=40,
    ATTITUDE_FILTER_REFERENCE=41,
    SET_PARAMETERS=50,
    SET_PARAMETERS_RES=51,
    SAVE_PARAMETERS=52,
    SAVE_PARAMETERS_RES=53,
    GET_PARAMETER=54,
    GET_PARAMETER_RES=55,
    CONTOLLER_ATTITUDE_SETPT=100,
    CONTOLLER_ATTITUDE_STATUS=101,
};



#include "rc_input.hpp"
NOP_EXTERNAL_STRUCTURE(rc_input_s,
    roll,
    pitch,
    yaw,
    throttle,
    switch_armed,
    switch_ap_control,
    (channel_raw, channel_raw_count),
    signal,
    rssi,
    timestamp);

#include "sensors.hpp"
NOP_EXTERNAL_STRUCTURE(quaternion_s, w, x, y, z);
NOP_EXTERNAL_STRUCTURE(imu_sample_t, angular_rate, accumulated_angle, linear_acceleration, timestamp);
NOP_EXTERNAL_STRUCTURE(barometer_sample_t, pressure, temperature, timestamp);
NOP_EXTERNAL_STRUCTURE(magnetometer_sample_t, magnetic_field, timestamp);

#include "control_loop.hpp"
NOP_EXTERNAL_STRUCTURE(control_status_t, mode);

#include "attitude_controller.hpp"
NOP_EXTERNAL_STRUCTURE(attitude_controller_input_t,
    attitude,
    angular_rate,
    angular_acceleration,
    acceleration,
    timestamp);
NOP_EXTERNAL_STRUCTURE(attitude_controller_status_t,
    angular_rate_ref,
    torque,
    timestamp);

#include "actuators.hpp"
NOP_EXTERNAL_STRUCTURE(actuators_t,
    actuators,
    actuators_len);

NOP_EXTERNAL_STRUCTURE(actuators_stamped_t,
    actuators,
    timestamp);

#include "attitude_filter.hpp"
NOP_EXTERNAL_STRUCTURE(attitude_filter_output_t,
    attitude,
    timestamp);
NOP_EXTERNAL_STRUCTURE(external_attitude_reference_t,
    attitude_reference,
    timestamp);

#endif /* ROS_INTERFACE_COMM_MSG_H */
