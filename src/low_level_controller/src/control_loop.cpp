#include <ch.h>
#include "parameter_storage.h"
#include "pid_with_parameter.hpp"
#include "rc_input.hpp"
#include "actuators.hpp"
#include "thread_prio.h"
#include "timestamp.h"

#include "control_loop.hpp"

msgbus::Topic<bool> output_armed;

static parameter_namespace_t control_ns;
static parameter_t control_loop_freq;
static parameter_t arm_remote_switch_channel;
static parameter_t arm_remote_switch_threshold;

bool arm_remote_switch_is_armed(const struct rc_input_s &rc_inputs)
{
    int ch = parameter_integer_read(&arm_remote_switch_channel);
    int ch_idx = ch-1;
    if (ch_idx >= 0 && ch_idx < rc_inputs.nb_channels) {
        float val = rc_inputs.channel[ch_idx];
        if (val > parameter_scalar_read(&arm_remote_switch_threshold)) {
            return true;
        } else {
            return false;
        }
    }
    return false;
}

static THD_WORKING_AREA(control_thread_wa, 1024);
static THD_FUNCTION(control_thread, arg) {

    (void)arg;
    chRegSetThreadName("control");
    uint32_t loop_period_us;

    auto sub_rc = msgbus::subscribe(rc_input);
    sub_rc.wait_for_update(); // make sure rc_input is valid

    timestamp_t last_rc_signal = 0;

    while (true) {
        if (parameter_changed(&control_loop_freq)) {
            float loop_frequency = parameter_scalar_read(&control_loop_freq);
            loop_period_us = 1e6/loop_frequency;
        }
        chThdSleepMicroseconds(loop_period_us);

        timestamp_t now = timestamp_get();
        struct rc_input_s rc_in = sub_rc.get_value();
        if (!rc_in.no_signal) {
            last_rc_signal = rc_in.timestamp;
        }

        if (arm_switch_is_armed() && arm_remote_switch_is_armed(rc_in)
            &&  timestamp_duration(last_rc_signal, now) < 1.5f) {
            std::array<float, NB_ACTUATORS> output;
            output[0] = rc_in.channel[0];
            actuators_set_output(output);
            output_armed.publish(true);
        } else {
            actuators_disable_all();
            output_armed.publish(false);
        }

    }
}

void control_init()
{
    parameter_namespace_declare(&control_ns, &parameters, "control");
    parameter_scalar_declare_with_default(&control_loop_freq, &control_ns, "loop_frequency", 100);
    parameter_integer_declare_with_default(&arm_remote_switch_channel, &control_ns, "arm_channel", 5);
    parameter_scalar_declare_with_default(&arm_remote_switch_threshold, &control_ns, "arm_threshold", -0.1);
    output_armed.publish(false);
}

void control_start()
{
    chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), THD_PRIO_CONTROL_LOOP, control_thread, NULL);
}
