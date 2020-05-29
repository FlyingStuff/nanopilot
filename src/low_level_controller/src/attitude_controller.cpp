#include <math.h>
#include "timestamp.h"
#include "attitude_controller.hpp"


msgbus::Topic<attitude_controller_input_t> attitude_controller_input_topic;
msgbus::Topic<attitude_controller_status_t> attitude_controller_status_topic;


AttitudeController::AttitudeController() :
m_ctrl_in_sub(attitude_controller_input_topic),
m_att_sub(attitude_filter_output_topic)
{
    declare_parameters(NULL);
}

void AttitudeController::declare_parameters(parameter_namespace_t *ns)
{
    parameter_namespace_declare(&m_namespace, ns, "attitude_controller");
    m_pid_controller_rpy[0].declare_parameters(&m_namespace, "roll_rate_controller");
    m_pid_controller_rpy[1].declare_parameters(&m_namespace, "pitch_rate_controller");
    m_pid_controller_rpy[2].declare_parameters(&m_namespace, "yaw_rate_controller");
    m_output_lp_rpy[0].declare_parameters(&m_namespace, "roll_low_pass_cutoff");
    m_output_lp_rpy[1].declare_parameters(&m_namespace, "pitch_low_pass_cutoff");
    m_output_lp_rpy[2].declare_parameters(&m_namespace, "yaw_low_pass_cutoff");
    parameter_scalar_declare_with_default(&m_rc_roll_rate_gain, &m_namespace, "rc_roll_rate_gain", 1);
    parameter_scalar_declare_with_default(&m_rc_pitch_rate_gain, &m_namespace, "rc_pitch_rate_gain", 1);
    parameter_scalar_declare_with_default(&m_rc_yaw_rate_gain, &m_namespace, "rc_yaw_rate_gain", 1);
    parameter_scalar_declare_with_default(&m_rc_acc_x_gain, &m_namespace, "rc_acc_x_gain", 0);
    parameter_scalar_declare_with_default(&m_rc_acc_y_gain, &m_namespace, "rc_acc_y_gain", 0);
    parameter_scalar_declare_with_default(&m_rc_acc_z_gain, &m_namespace, "rc_acc_z_gain", -1);

    parameter_vector_declare_with_default(&m_rpy_ctrl_mix[0], &m_namespace, "roll_mix", m_rpy_ctrl_mix_buf[0], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rpy_ctrl_mix[1], &m_namespace, "pitch_mix", m_rpy_ctrl_mix_buf[1], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rpy_ctrl_mix[2], &m_namespace, "yaw_mix", m_rpy_ctrl_mix_buf[2], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_force_ctrl_mix[0], &m_namespace, "fx_mix", m_force_ctrl_mix_buf[0], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_force_ctrl_mix[1], &m_namespace, "fy_mix", m_force_ctrl_mix_buf[1], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_force_ctrl_mix[2], &m_namespace, "fz_mix", m_force_ctrl_mix_buf[2], NB_ACTUATORS);
}

control_mode_t AttitudeController::process(const rc_input_s &rc_in, actuators_t &out)
{
    timestamp_t now = timestamp_get();
    attitude_controller_status_t status;
    status.timestamp = now;

    if (!m_att_sub.has_value()) {
        return CTRL_MODE_DISARMED;
    }
    auto att = m_att_sub.get_value();
    if (timestamp_duration(att.timestamp, now) > 0.01) {
        return CTRL_MODE_DISARMED;
    }

    control_mode_t mode = CTRL_MODE_MANUAL;
    if (rc_in.switch_ap_control) {
        mode = CTRL_MODE_AP;
    }
    if (mode == CTRL_MODE_AP && !m_ctrl_in_sub.has_value()) {
        mode = CTRL_MODE_MANUAL;
    }
    attitude_controller_input_t ctrl_in;
    if (mode == CTRL_MODE_AP) {
        ctrl_in = m_ctrl_in_sub.get_value();
        if (timestamp_duration(ctrl_in.timestamp, now) > 0.1) {
            mode = CTRL_MODE_MANUAL;
        }
        if (!att.reference_valid) {
            mode = CTRL_MODE_MANUAL; // AP control is only allowed if they share the same attitude reference
        }
    }
    if (mode == CTRL_MODE_MANUAL) {
        // rate mode
        ctrl_in.attitude.w = NAN;
        ctrl_in.attitude.x = NAN;
        ctrl_in.attitude.y = NAN;
        ctrl_in.attitude.z = NAN;
        ctrl_in.angular_rate[0] = parameter_scalar_read(&m_rc_roll_rate_gain) * rc_in.roll;
        ctrl_in.angular_rate[1] = parameter_scalar_read(&m_rc_pitch_rate_gain) * rc_in.pitch;
        ctrl_in.angular_rate[2] = parameter_scalar_read(&m_rc_yaw_rate_gain) * rc_in.yaw;
        ctrl_in.angular_acceleration[0] = 0;
        ctrl_in.angular_acceleration[1] = 0;
        ctrl_in.angular_acceleration[2] = 0;
        ctrl_in.acceleration[0] = parameter_scalar_read(&m_rc_acc_x_gain) * rc_in.throttle;
        ctrl_in.acceleration[1] = parameter_scalar_read(&m_rc_acc_y_gain) * rc_in.throttle;
        ctrl_in.acceleration[2] = parameter_scalar_read(&m_rc_acc_z_gain) * rc_in.throttle;
    }

    // TODO attitude control
    (void) ctrl_in.attitude;
    status.angular_rate_ref[0] = ctrl_in.angular_rate[0];
    status.angular_rate_ref[1] = ctrl_in.angular_rate[1];
    status.angular_rate_ref[2] = ctrl_in.angular_rate[2];

    // TODO add mass, inertia parameters to have correct units

    // angular rate PID control
    for (int i = 0; i < 3; i++){
        if (isfinite(status.angular_rate_ref[i])) {
            float rate_error = att.angular_rate[i] - status.angular_rate_ref[i];
            status.torque[i] = m_pid_controller_rpy[i].process(rate_error);
            status.torque[i] = m_output_lp_rpy[i].process(status.torque[i]);
            // TODO add angular_acceleration
            (void) ctrl_in.angular_acceleration[i];
        } else {
            status.torque[i] = 0;
            m_pid_controller_rpy[i].reset();
        }
    }


    out.actuators_len = NB_ACTUATORS;
    // torque
    std::array<float, NB_ACTUATORS> coeff;
    for (int axis=0; axis < 3; axis++) {
        parameter_vector_read(&m_rpy_ctrl_mix[axis], coeff.data());
        for (int i = 0; i < NB_ACTUATORS; i++) {
            out.actuators[i] = coeff[i] * status.torque[axis];
        }
    }
    // force
    for (int axis=0; axis < 3; axis++) {
        parameter_vector_read(&m_force_ctrl_mix[axis], coeff.data());
        for (int i = 0; i < NB_ACTUATORS; i++) {
            out.actuators[i] += coeff[i] * ctrl_in.acceleration[axis];
        }
    }

    attitude_controller_status_topic.publish(status);

    return mode;
}

void AttitudeController::notify_output_disabled()
{
    for (int i = 0; i < 3; i++) {
        m_pid_controller_rpy[i].reset();
    }
}

void AttitudeController::set_update_frequency(float freq)
{
    for (int i = 0; i < 3; i++) {
        m_pid_controller_rpy[i].set_update_frequency(freq);
        m_output_lp_rpy[i].set_update_frequency(freq);
    }
}

timestamp_t AttitudeController::ap_control_signal_timestamp()
{
    if (!m_ctrl_in_sub.has_value()) {
        return 0;
    }
    return m_ctrl_in_sub.get_value().timestamp;
}



