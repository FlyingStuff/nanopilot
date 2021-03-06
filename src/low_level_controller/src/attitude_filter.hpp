#ifndef ATTITUDE_FILTER_HPP
#define ATTITUDE_FILTER_HPP

/*
 * This filter takes IMU rate(+delta angles) and acceleration data and outputs attitude
 * If available an external attitude reference is used, otherwise the attitude
 * drift is stabilized in roll & pitch using acceleration.
 */

#include "msgbus/msgbus.hpp"
#include "timestamp.h"
#include "sensors.hpp"

typedef struct {
    quaternion_s attitude_reference;
    timestamp_t timestamp;
} external_attitude_reference_t;

typedef struct {
    float angular_rate[3]; // body frame
    quaternion_s attitude;
    bool reference_valid;
    timestamp_t timestamp;
} attitude_filter_output_t;

extern msgbus::Topic<attitude_filter_output_t> attitude_filter_output_topic;
extern msgbus::Topic<external_attitude_reference_t> attitude_reference_topic;

void attitude_filter_init();
void attitude_filter_update();

#endif /* ATTITUDE_FILTER_HPP */
