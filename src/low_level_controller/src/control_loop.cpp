#include <ch.h>
#include "parameter_storage.h"
#include "pid_with_parameter.hpp"
#include "rc_input.hpp"
#include "actuators.hpp"
#include "thread_prio.h"
#include "timestamp.h"

#include "control_loop.hpp"

msgbus::Topic<bool> output_armed;
static RateController *s_rate_controller;
static RCMixer *s_rc_mixer;

static parameter_namespace_t control_ns;
static parameter_t control_loop_freq;
static parameter_namespace_t rc_ns;
static parameter_t arm_remote_switch_channel;
static parameter_t arm_remote_switch_threshold;
static parameter_t roll_input_channel;
static parameter_t pitch_input_channel;
static parameter_t yaw_input_channel;
static parameter_t roll_rate_gain_param;
static parameter_t pitch_rate_gain_param;
static parameter_t yaw_rate_gain_param;


static bool arm_remote_switch_is_armed(const struct rc_input_s &rc_inputs)
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

static float get_rc_channel_value(const struct rc_input_s &rc_inputs, int channel)
{
    int index = channel - 1;
    if (index < 0 || index >= rc_inputs.nb_channels) {
        return 0;
    }
    return rc_inputs.channel[index];
}

static void get_rc_rate_inputs(const struct rc_input_s &rc_inputs, float rate_setpoint_rpy[3])
{
    int roll_ch = parameter_integer_read(&roll_input_channel);
    int pitch_ch = parameter_integer_read(&pitch_input_channel);
    int yaw_ch = parameter_integer_read(&yaw_input_channel);

    float roll_gain = parameter_scalar_read(&roll_rate_gain_param);
    float pitch_gain = parameter_scalar_read(&pitch_rate_gain_param);
    float yaw_gain = parameter_scalar_read(&yaw_rate_gain_param);

    rate_setpoint_rpy[0] = - roll_gain * get_rc_channel_value(rc_inputs, roll_ch);
    rate_setpoint_rpy[1] = - pitch_gain * get_rc_channel_value(rc_inputs, pitch_ch);
    rate_setpoint_rpy[2] = - yaw_gain * get_rc_channel_value(rc_inputs, yaw_ch);
}

static THD_WORKING_AREA(control_thread_wa, 1024);
static THD_FUNCTION(control_thread, arg)
{
    (void)arg;
    chRegSetThreadName("control");
    uint32_t loop_period_us=0;

    auto sub_rc = msgbus::subscribe(rc_input);
    sub_rc.wait_for_update(); // make sure rc_input is valid

    timestamp_t last_rc_signal = 0;

    while (true) {
        if (parameter_changed(&control_loop_freq)) {
            float loop_frequency = parameter_scalar_read(&control_loop_freq);
            loop_period_us = 1e6/loop_frequency;
            s_rate_controller->set_update_frequency(loop_frequency);
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

            float rate_setpoint_rpy[3];
            get_rc_rate_inputs(rc_in, rate_setpoint_rpy);
            float rate_measured_rpy[3];
            float rate_ctrl_output[3];

            s_rate_controller->process(rate_setpoint_rpy, rate_measured_rpy, rate_ctrl_output);
            s_rc_mixer->mix(rate_ctrl_output, rc_in, output);

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
    parameter_namespace_declare(&rc_ns, &parameters, "rc");
    parameter_integer_declare_with_default(&arm_remote_switch_channel, &rc_ns, "arm_channel", 5);
    parameter_scalar_declare_with_default(&arm_remote_switch_threshold, &rc_ns, "arm_threshold", -0.1);
    parameter_integer_declare_with_default(&roll_input_channel, &rc_ns, "roll_channel", 1);
    parameter_integer_declare_with_default(&pitch_input_channel, &rc_ns, "pitch_channel", 2);
    parameter_integer_declare_with_default(&yaw_input_channel, &rc_ns, "yaw_channel", 3);
    parameter_scalar_declare_with_default(&roll_rate_gain_param, &rc_ns, "roll_rate_gain", 2*3.14f);
    parameter_scalar_declare_with_default(&pitch_rate_gain_param, &rc_ns, "pitch_rate_gain", 2*3.14f);
    parameter_scalar_declare_with_default(&yaw_rate_gain_param, &rc_ns, "yaw_rate_gain", 2*3.14f);
    output_armed.publish(false);
}

void control_start(RateController &rate_ctrl, RCMixer &rc_mixer)
{
    s_rate_controller = &rate_ctrl;
    s_rc_mixer = &rc_mixer;
    chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), THD_PRIO_CONTROL_LOOP, control_thread, NULL);
}
