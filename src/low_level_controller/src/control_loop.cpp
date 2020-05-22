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
msgbus::Topic<uint64_t> ap_control_latency_topic;
msgbus::Topic<struct ap_ctrl_s> ap_ctrl;
msgbus::Topic<std::array<float, NB_ACTUATORS>> actuator_output_topic;
msgbus::Topic<std::array<float, 3>> rate_setpoint_rpy_topic;
msgbus::Topic<std::array<float, 3>> rate_measured_rpy_topic;
msgbus::Topic<std::array<float, 3>> rate_ctrl_output_rpy_topic;
static RateController *s_rate_controller;
static OutputMixer *s_output_mixer;

parameter_namespace_t control_ns;
static parameter_t control_loop_freq;
static parameter_t R_board_to_body_param;


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
    auto imu_sub = msgbus::subscribe(imu);
    imu_sub.wait_for_update(); // make sure gyro is valid
    auto sub_ap_ctrl = msgbus::subscribe(ap_ctrl);

    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_board_to_body;
    while (true) {
        if (parameter_changed(&control_loop_freq)) {
            float loop_frequency = parameter_scalar_get(&control_loop_freq);
            loop_period_us = 1e6/loop_frequency;
            s_rate_controller->set_update_frequency(loop_frequency);
            s_output_mixer->set_update_frequency(loop_frequency);
        }
        if (parameter_changed(&R_board_to_body_param)) {
            parameter_vector_get(&R_board_to_body_param, R_board_to_body.data());
        }

        chThdSleepMicroseconds(loop_period_us);

        timestamp_t now = timestamp_get();

        struct rc_input_raw_s rc_in_raw = sub_rc.get_value();
        struct rc_input_s rc_in;
        rc_input_decode(rc_in_raw, rc_in);
        invalidate_rc_signal_if_older_than(rc_in, now, 1.5f);
        rc_input_topic.publish(rc_in);

        imu_sample_t imu = imu_sub.get_value();

        static bool was_armed = true;
        if (!arm_switch_is_armed()) {
            if (was_armed) {
                actuators_disable_all(); // only run once so we can still run the esc calibration shell command
                was_armed = false;
            }
            output_armed_topic.publish(false);
            continue;
        } else {
            was_armed = true;
        }
        if (rc_in.switch_armed
            && rc_in.signal
            && timestamp_duration(imu.timestamp, now) < 0.01f) {

            std::array<float, NB_ACTUATORS> output;
            std::array<float, 3> rate_setpoint_rpy;
            std::array<float, 3> rate_measured_rpy;
            std::array<float, 3> rate_ctrl_output;
            rate_setpoint_rpy[0] = rc_in.roll;
            rate_setpoint_rpy[1] = rc_in.pitch;
            rate_setpoint_rpy[2] = rc_in.yaw;

            bool ap_in_control = false;
            bool ap_timeout = false;
            struct ap_ctrl_s ap_ctrl_msg;
            if (rc_in.switch_ap_control) {
                if (sub_ap_ctrl.has_value()) {
                    ap_ctrl_msg = sub_ap_ctrl.get_value();
                    ap_control_latency_topic.publish(timestamp_duration_ns(ap_ctrl_msg.timestamp, now));
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
            Eigen::Map<Eigen::Vector3f> rate_measured_board(imu.angular_rate);
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
            std::fill(output.begin(), output.end(), 0);
            actuators_set_output(output);
            actuator_output_topic.publish(output);
            output_armed_topic.publish(false);
            ap_in_control_topic.publish(false);
        }

    }
}



void control_init()
{
    parameter_namespace_declare(&control_ns, &parameters, "control");
    parameter_scalar_declare_with_default(&control_loop_freq, &control_ns, "loop_frequency", 100);

    rc_input_init(&parameters);

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
