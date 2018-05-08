#ifndef SUMD_INPUT_H
#define SUMD_INPUT_H

#include <ch.h>

#define RC_INPUT_MAX_NB_CHANNELS 10

struct rc_input_s {
    uint8_t nb_channels;
    bool no_signal;
    float channel[RC_INPUT_MAX_NB_CHANNELS];
};

void sumd_input_start(BaseSequentialStream *input);
void sumd_input_get(struct rc_input_s *rc_in);

#endif /* SUMD_INPUT_H */
