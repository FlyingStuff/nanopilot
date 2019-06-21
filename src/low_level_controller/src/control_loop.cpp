#include <ch.h>
#include "parameter_storage.h"
#include "pid_with_parameter.hpp"
#include "rc_input.hpp"
#include "actuators.hpp"
#include "thread_prio.h"
#include "timestamp.h"
#include "sensors.hpp"

#include "control_loop.hpp"

msgbus::Topic<bool> output_armed_topic;
msgbus::Topic<bool> ap_in_control_topic;
msgbus::Topic<bool> ap_control_timeout;
msgbus::Topic<struct ap_ctrl_s> ap_ctrl;
msgbus::Topic<std::array<float, NB_ACTUATORS>> actuator_output_topic;
msgbus::Topic<std::array<float, 3>> rate_setpoint_rpy_topic;
msgbus::Topic<std::array<float, 3>> rate_measured_rpy_topic;
msgbus::Topic<std::array<float, 3>> rate_ctrl_output_rpy_topic;
static RateController *s_rate_controller;
static OutputMixer *s_output_mixer;

parameter_namespace_t control_ns;
static parameter_t control_loop_freq;
static parameter_namespace_t rc_ns;
static parameter_t arm_remote_switch_channel;
static parameter_t arm_remote_switch_threshold;
static parameter_t ap_remote_switch_channel;
static parameter_t ap_remote_switch_threshold;
static parameter_t roll_input_channel;
static parameter_t pitch_input_channel;
static parameter_t yaw_input_channel;
static parameter_t roll_rate_gain_param;
static parameter_t pitch_rate_gain_param;
static parameter_t yaw_rate_gain_param;
static parameter_t R_board_to_body_param;


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

static bool ap_control_switch_is_on(const struct rc_input_s &rc_inputs)
{
    int ch = parameter_integer_read(&ap_remote_switch_channel);
    int ch_idx = ch-1;
    if (ch_idx >= 0 && ch_idx < rc_inputs.nb_channels) {
        float val = rc_inputs.channel[ch_idx];
        if (val > parameter_scalar_read(&ap_remote_switch_threshold)) {
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

static THD_WORKING_AREA(control_thread_wa, 2000);
static THD_FUNCTION(control_thread, arg)
{
    (void)arg;
    chRegSetThreadName("control");
    uint32_t loop_period_us=0;

    auto sub_rc = msgbus::subscribe(rc_input);
    sub_rc.wait_for_update(); // make sure rc_input is valid
    auto sub_gyro = msgbus::subscribe(rate_gyro);
    sub_gyro.wait_for_update(); // make sure gyro is valid
    auto sub_ap_ctrl = msgbus::subscribe(ap_ctrl);

    timestamp_t last_rc_signal = 0;

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_board_to_body;
    while (true) {
        if (parameter_changed(&control_loop_freq)) {
            float loop_frequency = parameter_scalar_get(&control_loop_freq);
            loop_period_us = 1e6/loop_frequency;
            s_rate_controller->set_update_frequency(loop_frequency);
        }
        if (parameter_changed(&R_board_to_body_param)) {
            parameter_vector_get(&R_board_to_body_param, R_board_to_body.data());
        }

        chThdSleepMicroseconds(loop_period_us);

        timestamp_t now = timestamp_get();
        struct rc_input_s rc_in = sub_rc.get_value();
        if (!rc_in.no_signal) {
            last_rc_signal = rc_in.timestamp;
        }
        rate_gyro_sample_t gyro = sub_gyro.get_value();

        static bool was_armed = true;
        if (!arm_switch_is_armed()) {
            if (was_armed) {
                actuators_disable_all();
                was_armed = false;
            }
            output_armed_topic.publish(false);
            continue;
        } else {
            was_armed = true;
        }
        if (arm_remote_switch_is_armed(rc_in)
            && timestamp_duration(last_rc_signal, now) < 1.5f
            && timestamp_duration(gyro.timestamp, now) < 0.01f) {

            std::array<float, NB_ACTUATORS> output;
            std::array<float, 3> rate_setpoint_rpy;
            std::array<float, 3> rate_measured_rpy;
            std::array<float, 3> rate_ctrl_output;
            get_rc_rate_inputs(rc_in, rate_setpoint_rpy.data());

            bool ap_in_control = false;
            bool ap_timeout = false;
            bool ap_ctrl_en = ap_control_switch_is_on(rc_in);
            struct ap_ctrl_s ap_ctrl_msg;
            if (ap_ctrl_en) {
                if (sub_ap_ctrl.has_value()) {
                    ap_ctrl_msg = sub_ap_ctrl.get_value();
                    if (fabsf(timestamp_duration(ap_ctrl_msg.timestamp, now)) < 0.1f) {
                        rate_setpoint_rpy = ap_ctrl_msg.rate_setpoint_rpy;
                        ap_in_control = true;
                    } else {
                        ap_timeout = true;
                    }
                } else {
                    ap_timeout = true;
                }
            }
            ap_control_timeout.publish(ap_timeout);

            // transform rate to body frame
            Eigen::Map<Eigen::Vector3f> rate_measured_board(gyro.rate);
            Eigen::Map<Eigen::Vector3f> rate_measured_rpy_vect(rate_measured_rpy.data());
            rate_measured_rpy_vect = R_board_to_body * rate_measured_board;

            s_rate_controller->process(rate_setpoint_rpy.data(),
                                        rate_measured_rpy.data(),
                                        rate_ctrl_output.data());
            s_output_mixer->mix(rate_ctrl_output.data(), rc_in, ap_ctrl_msg, ap_in_control, output);

            actuators_set_output(output);

            output_armed_topic.publish(true);
            rate_setpoint_rpy_topic.publish(rate_setpoint_rpy);
            rate_measured_rpy_topic.publish(rate_measured_rpy);
            rate_ctrl_output_rpy_topic.publish(rate_ctrl_output);
            actuator_output_topic.publish(output);
            ap_in_control_topic.publish(ap_in_control);
        } else {
            s_rate_controller->reset();
            std::array<float, NB_ACTUATORS> output;
            for (float &o: output) {
                o = 0;
            }
            actuators_set_output(output);
            actuator_output_topic.publish(output);
            output_armed_topic.publish(false);
        }

    }
}



void control_init()
{
    parameter_namespace_declare(&control_ns, &parameters, "control");
    parameter_scalar_declare_with_default(&control_loop_freq, &control_ns, "loop_frequency", 100);
    parameter_namespace_declare(&rc_ns, &parameters, "rc");
    parameter_integer_declare_with_default(&arm_remote_switch_channel, &rc_ns, "arm_channel", 5);
    parameter_scalar_declare_with_default(&arm_remote_switch_threshold, &rc_ns, "arm_threshold", 0.1);
    parameter_integer_declare_with_default(&ap_remote_switch_channel, &rc_ns, "ap_switch_channel", -1);
    parameter_scalar_declare_with_default(&ap_remote_switch_threshold, &rc_ns, "ap_switch_threshold", 0.1);
    parameter_integer_declare_with_default(&roll_input_channel, &rc_ns, "roll_channel", 2);
    parameter_integer_declare_with_default(&pitch_input_channel, &rc_ns, "pitch_channel", 3);
    parameter_integer_declare_with_default(&yaw_input_channel, &rc_ns, "yaw_channel", 4);
    parameter_scalar_declare_with_default(&roll_rate_gain_param, &rc_ns, "roll_rate_gain", 2*3.14f);
    parameter_scalar_declare_with_default(&pitch_rate_gain_param, &rc_ns, "pitch_rate_gain", 2*3.14f);
    parameter_scalar_declare_with_default(&yaw_rate_gain_param, &rc_ns, "yaw_rate_gain", 2*3.14f);

    static float R_board_to_body[9] = {1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1};
    parameter_vector_declare_with_default(&R_board_to_body_param, &control_ns, "R_board_to_body", R_board_to_body, 9);
    output_armed_topic.publish(false);
}

void control_start(RateController &rate_ctrl, OutputMixer &output_mixer)
{
    s_rate_controller = &rate_ctrl;
    s_output_mixer = &output_mixer;
    chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa), THD_PRIO_CONTROL_LOOP, control_thread, NULL);
}
