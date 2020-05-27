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
    // RATE_CTRL_SETPOINT_RPY=12,
    // RATE_CTRL_MEASURED_RPY=13,
    // RATE_CTRL_OUTPUT_RPY=14,
    // AP_CONTROL=15,
    // OUTPUT_IS_ARMED=16,
    // AP_IN_CONTROL=17,
    IMU=20,
    MAGNETOMETER=22,
    BAROMETER=23,
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

// const int MAX_NB_ACTUATORS=16;
// struct ap_ctrl_s {
//     std::array<float, 3> rate_setpoint_rpy{{0, 0, 0}};
//     std::array<float, MAX_NB_ACTUATORS> direct_output{{0}};
//     std::array<float, 3> feed_forward_torque_rpy{{0, 0, 0}};
//     std::array<float, 3> force_xyz{{0, 0, 0}};
//     timestamp_t timestamp{0};
// };
// NOP_EXTERNAL_STRUCTURE(ap_ctrl_s, rate_setpoint_rpy, direct_output, feed_forward_torque_rpy, force_xyz, timestamp);

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


#endif /* ROS_INTERFACE_COMM_MSG_H */
