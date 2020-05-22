#ifndef RC_INPUT_HPP
#define RC_INPUT_HPP

#include "timestamp.h"
#include "msgbus/msgbus.hpp"
#include <parameter/parameter.h>

#define RC_INPUT_MAX_NB_CHANNELS 10

struct rc_input_raw_s {
    float channel[RC_INPUT_MAX_NB_CHANNELS];
    uint8_t channel_count;
    bool signal;
    timestamp_t timestamp;
};

struct rc_input_s {
    float roll;
    float pitch;
    float yaw;
    float throttle;
    bool switch_armed;
    bool switch_ap_control;

    float channel_raw[RC_INPUT_MAX_NB_CHANNELS];
    uint8_t channel_raw_count;
    bool signal;
    float rssi;
    timestamp_t timestamp;
};

extern msgbus::Topic<struct rc_input_raw_s> rc_input_raw_topic;
extern msgbus::Topic<struct rc_input_s> rc_input_topic;

void rc_input_init(parameter_namespace_t *param_namespace);
void rc_input_decode(const struct rc_input_raw_s &raw, struct rc_input_s &out);

#endif /* RC_INPUT_HPP */
