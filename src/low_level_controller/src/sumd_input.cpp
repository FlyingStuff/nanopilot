#include <ch.h>
#include <hal.h>
#include "thread_prio.h"
#include "hott/sumd.h"
#include "log.h"
#include "rc_input.hpp"
#include "sumd_input.hpp"

static THD_WORKING_AREA(sumd_input_task_wa, 1024);
static THD_FUNCTION(sumd_input_task, arg)
{
    chRegSetThreadName("rc sumd input");
    BaseSequentialStream *input = (BaseSequentialStream*)arg;
    struct sumd_receiver_s rc;
    sumd_receiver_init(&rc);
    while (1) {
        char c = streamGet(input);
        int ret = sumd_receive(&rc, c);
        if (ret == SUMD_RECEIVE_COMPLETE) {
            struct rc_input_s in;
            in.no_signal = rc.no_signal_failsafe;
            if (rc.nb_channels > RC_INPUT_MAX_NB_CHANNELS) {
                in.nb_channels = RC_INPUT_MAX_NB_CHANNELS;
            } else {
                in.nb_channels = rc.nb_channels;
            }
            int i;
            for (i = 0; i < in.nb_channels; i++) {
                in.channel[i] = ((float)rc.channel[i] - SUMD_POS_NEUTRAL) / (SUMD_POS_HIGH - SUMD_POS_NEUTRAL);
            }
            in.timestamp = timestamp_get();
            rc_input.publish(in);
        }
    }
}

void sumd_input_start(BaseSequentialStream *input)
{
    chThdCreateStatic(sumd_input_task_wa, sizeof(sumd_input_task_wa), THD_PRIO_RC_SUMD_IN, sumd_input_task, input);
}
