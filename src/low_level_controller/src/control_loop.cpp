#include <ch.h>
#include "parameter_storage.h"
#include "pid_with_parameter.hpp"
#include "rc_input.hpp"
#include "actuators.hpp"
#include "thread_prio.h"
#include "timestamp.h"
#include "sensors.hpp"
#include "attitude_filter.hpp"

#include "control_loop.hpp"

msgbus::Topic<control_status_t> control_status_topic;
msgbus::Topic<std::array<float, NB_ACTUATORS>> actuator_output_topic;
msgbus::Topic<float> ap_control_latency_topic;
static ControllerInterface *s_controller=NULL;

parameter_namespace_t control_ns;
static parameter_t control_loop_freq;


void invalidate_rc_signal_if_older_than(struct rc_input_s &rc_in, timestamp_t now, float max_age)
{
    if (timestamp_duration(rc_in.timestamp, now) > max_age) {
        rc_in.signal = false;
    }
}


static THD_WORKING_AREA(control_thread_wa, 2000);
static THD_FUNCTION(control_thread, arg)
{
    (void)arg;
    chRegSetThreadName("control");
    uint32_t loop_period_us=0;

    auto sub_rc = msgbus::subscribe(rc_input_raw_topic);
    sub_rc.wait_for_update(); // make sure rc_input is valid

    while (true) {
        if (parameter_changed(&control_loop_freq)) {
            float loop_frequency = parameter_scalar_get(&control_loop_freq);
            loop_period_us = 1e6/loop_frequency;
            s_controller->set_update_frequency(loop_frequency);
        }

        chThdSleepMicroseconds(loop_period_us);

        timestamp_t now = timestamp_get();

        struct rc_input_raw_s rc_in_raw = sub_rc.get_value();
        struct rc_input_s rc_in;
        rc_input_decode(rc_in_raw, rc_in);
        invalidate_rc_signal_if_older_than(rc_in, now, 1.5f);
        rc_input_topic.publish(rc_in);

        attitude_filter_update();

        std::array<float, NB_ACTUATORS> output;
        std::fill(output.begin(), output.end(), 0);
        control_mode_t mode;
        bool ap_timeout = false;
        if (!arm_switch_is_armed() || !rc_in.switch_armed || !rc_in.signal) {
            s_controller->notify_output_disabled();
            mode = CTRL_MODE_DISARMED;
        } else {
            mode = s_controller->process(rc_in, output);
            if (rc_in.switch_ap_control && mode != CTRL_MODE_AP) {
                ap_timeout = true;
            }
        }

        static bool was_armed = true;
        if (arm_switch_is_armed()) {
            actuators_set_output(output);
            was_armed = true;
        } else {
            if (was_armed) {
                actuators_disable_all();
            }
            was_armed = false;
        }

        actuator_output_topic.publish(output);
        control_status_t status;
        status.mode = mode;
        status.ap_timeout = ap_timeout;
        control_status_topic.publish(status);
        float ap_latency = timestamp_duration(s_controller->ap_control_signal_timestamp(), now);
        ap_control_latency_topic.publish(ap_latency);
    }
}



void control_init()
{
    parameter_namespace_declare(&control_ns, &parameters, "control");
    parameter_scalar_declare_with_default(&control_loop_freq, &control_ns, "loop_frequency", 100);

    rc_input_init(&parameters);

    attitude_filter_init();
}

void control_start(ControllerInterface &controller)
{
    s_controller = &controller;
    chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), THD_PRIO_CONTROL_LOOP, control_thread, NULL);
}
