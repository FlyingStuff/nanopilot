#ifndef SUMD_INPUT_H
#define SUMD_INPUT_H

#include <ch.h>
#include <hal.h>
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

void sumd_input_start(BaseSequentialStream *input);

#endif /* SUMD_INPUT_H */
