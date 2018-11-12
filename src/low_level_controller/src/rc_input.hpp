#ifndef RC_INPUT_HPP
#define RC_INPUT_HPP

#include "timestamp.h"
#include "msgbus/msgbus.hpp"

#define RC_INPUT_MAX_NB_CHANNELS 10

struct rc_input_s {
    uint8_t nb_channels;
    bool no_signal;
    float channel[RC_INPUT_MAX_NB_CHANNELS];
    timestamp_t timestamp;
};

extern msgbus::Topic<struct rc_input_s> rc_input;

#endif /* RC_INPUT_HPP */
