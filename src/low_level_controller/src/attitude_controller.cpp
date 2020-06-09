#include <math.h>
#include "timestamp.h"
#include <Eigen/Dense>
#include "attitude_controller.hpp"


msgbus::Topic<attitude_controller_input_t> attitude_controller_input_topic;
msgbus::Topic<attitude_controller_status_t> attitude_controller_status_topic;


/* split a rotation such that:
 * rotation = rotation_twist * rotation_tilt
 * (first tilt, then twist)
 * where rotation_twist is around twist_axis and rotation_tilt is around a
 * perpendicular axis
 * output quaternions have positive real part
 */
void rotation_split_tilt_twist(const Eigen::Quaternionf &rotation,
    const Eigen::Vector3f &twist_axis,
    Eigen::Quaternionf &rotation_tilt,
    Eigen::Quaternionf &rotation_twist)
{
    auto q = rotation.normalized();
    if (q.w() < 0) { // fix quaternion sign to have positive real part
        q.coeffs() = -q.coeffs();
    }
    auto t = twist_axis.normalized();
    double qv_dot_t = q.vec().dot(t);
    double s = 1/sqrt(qv_dot_t*qv_dot_t + q.w()*q.w());
    rotation_twist.w() = q.w() * s;
    auto b = qv_dot_t * s;
    rotation_twist.vec() = b * t;
    rotation_tilt = rotation_twist.conjugate()*q;
    if (rotation_tilt.w() < 0) {
        rotation_tilt.coeffs() = -rotation_tilt.coeffs();
    }
}

Eigen::Quaternionf shortest_rotation_quaternion(
    const Eigen::Vector3f &v1,
    const Eigen::Vector3f &v2); // defined in attitude_filter.cpp TODO


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
    parameter_scalar_declare_with_default(&m_attitude_roll_time_cst, &m_namespace, "attitude_roll_time_cst", 0.1);
    parameter_scalar_declare_with_default(&m_attitude_pitch_time_cst, &m_namespace, "attitude_pitch_time_cst", 0.1);
    parameter_scalar_declare_with_default(&m_attitude_yaw_time_cst, &m_namespace, "attitude_yaw_time_cst", 0.5);
    parameter_boolean_declare_with_default(&m_attitude_tilt_twist_decompostition, &m_namespace, "attitude_error_tilt_twist_decomposition", true);
    parameter_scalar_declare_with_default(&m_max_roll_rate, &m_namespace, "max_roll_rate", 3.14);
    parameter_scalar_declare_with_default(&m_max_pitch_rate, &m_namespace, "max_pitch_rate", 3.14);
    parameter_scalar_declare_with_default(&m_max_yaw_rate, &m_namespace, "max_yaw_rate", 3.14);
    parameter_scalar_declare_with_default(&m_Ixx, &m_namespace, "Ixx", 1);
    parameter_scalar_declare_with_default(&m_Iyy, &m_namespace, "Iyy", 1);
    parameter_scalar_declare_with_default(&m_Izz, &m_namespace, "Izz", 1);
    parameter_scalar_declare_with_default(&m_Ixy, &m_namespace, "Ixy", 0);
    parameter_scalar_declare_with_default(&m_Ixz, &m_namespace, "Ixz", 0);
    parameter_scalar_declare_with_default(&m_Iyz, &m_namespace, "Iyz", 0);
    parameter_scalar_declare_with_default(&m_mass, &m_namespace, "mass", 1);
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
    if (timestamp_duration(att.timestamp, now) > 0.01f) {
        return CTRL_MODE_DISARMED; // IMU timeout
    }
    Eigen::Quaternionf att_estimate(att.attitude.w,
            att.attitude.x,
            att.attitude.y,
            att.attitude.z);

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
        if (timestamp_duration(ctrl_in.timestamp, now) > 0.1f) {
            mode = CTRL_MODE_MANUAL; // AP control message timeout
        }
        if (!att.reference_valid) {
            mode = CTRL_MODE_MANUAL; // AP control is only allowed if they share the same attitude reference
        }
    }
    if (mode == CTRL_MODE_MANUAL) {
        if (rc_in.channel_raw[5] < 0.1f) { // TODO make configurable
            // rate mode
            ctrl_in.attitude.w = NAN;
            ctrl_in.attitude.x = NAN;
            ctrl_in.attitude.y = NAN;
            ctrl_in.attitude.z = NAN;
            ctrl_in.angular_rate[0] = parameter_scalar_read(&m_rc_roll_rate_gain) * rc_in.roll;
            ctrl_in.angular_rate[1] = parameter_scalar_read(&m_rc_pitch_rate_gain) * rc_in.pitch;
            ctrl_in.angular_rate[2] = parameter_scalar_read(&m_rc_yaw_rate_gain) * rc_in.yaw;
        } else {
            // attitude mode
            // Y is yaw aligned level frame
            Eigen::Vector3f fwd_dir_I = att_estimate.toRotationMatrix() * Eigen::Vector3f::UnitX();
            fwd_dir_I[2] = 0; // project to xy
            Eigen::Quaternionf q_y_to_I = shortest_rotation_quaternion(Eigen::Vector3f::UnitX(), fwd_dir_I);
            Eigen::Vector3f accel_d(0, 0, -1); // up
            Eigen::Vector3f accel_Y(-rc_in.pitch, rc_in.roll, -1);
            auto desired_to_Y = shortest_rotation_quaternion(accel_d, accel_Y);
            auto desired_att = q_y_to_I * desired_to_Y;
            ctrl_in.attitude.w = desired_att.w();
            ctrl_in.attitude.x = desired_att.x();
            ctrl_in.attitude.y = desired_att.y();
            ctrl_in.attitude.z = desired_att.z();
            ctrl_in.angular_rate[0] = 0;
            ctrl_in.angular_rate[1] = 0;
            ctrl_in.angular_rate[2] = parameter_scalar_read(&m_rc_yaw_rate_gain) * rc_in.yaw;
        }

        // common
        ctrl_in.angular_acceleration[0] = 0;
        ctrl_in.angular_acceleration[1] = 0;
        ctrl_in.angular_acceleration[2] = 0;
        ctrl_in.acceleration[0] = parameter_scalar_read(&m_rc_acc_x_gain) * rc_in.throttle;
        ctrl_in.acceleration[1] = parameter_scalar_read(&m_rc_acc_y_gain) * rc_in.throttle;
        ctrl_in.acceleration[2] = parameter_scalar_read(&m_rc_acc_z_gain) * rc_in.throttle;
    }

    status.attitude_setpt = ctrl_in.attitude;
    if (isfinite(ctrl_in.attitude.w)) { // attitude control
        status.attitude_control = true;
        Eigen::Quaternionf att_setpt(ctrl_in.attitude.w,
            ctrl_in.attitude.x,
            ctrl_in.attitude.y,
            ctrl_in.attitude.z);
        auto att_error = (att_setpt.conjugate()*att_estimate).normalized();
        if (att_error.w() < 0) { // fix attitude error to have positive real part
            att_error.coeffs() = -att_error.coeffs();
        }

        Eigen::Vector3f att_error_vec;
        if (parameter_boolean_read(&m_attitude_tilt_twist_decompostition)) {
            Eigen::Vector3f twist_axis(0, 0, 1); // twist axis = yaw
            Eigen::Quaternionf att_error_twist, att_error_tilt;
            rotation_split_tilt_twist(att_error, twist_axis, att_error_tilt, att_error_twist);
            att_error_vec = 2*(att_error_tilt.vec() + att_error_twist.vec());
        } else {
            att_error_vec = 2*att_error.vec();
        }
        Eigen::Vector3f K(1/parameter_scalar_read(&m_attitude_roll_time_cst),
            1/parameter_scalar_read(&m_attitude_pitch_time_cst),
            1/parameter_scalar_read(&m_attitude_yaw_time_cst));
        Eigen::Vector3f rate_setpt = -K.cwiseProduct(att_error_vec);

        // limit max angular rate
        Eigen::Vector3f max_rate_setpt(parameter_scalar_read(&m_max_roll_rate),
            parameter_scalar_read(&m_max_pitch_rate),
            parameter_scalar_read(&m_max_yaw_rate));
        rate_setpt = rate_setpt.cwiseMin(max_rate_setpt).cwiseMax(-max_rate_setpt);
        // add feedforward
        status.angular_rate_ref[0] = rate_setpt[0] + ctrl_in.angular_rate[0];
        status.angular_rate_ref[1] = rate_setpt[1] + ctrl_in.angular_rate[1];
        status.angular_rate_ref[2] = rate_setpt[2] + ctrl_in.angular_rate[2];
    } else {
        status.attitude_control = false;
        status.angular_rate_ref[0] = ctrl_in.angular_rate[0];
        status.angular_rate_ref[1] = ctrl_in.angular_rate[1];
        status.angular_rate_ref[2] = ctrl_in.angular_rate[2];
    }


    Eigen::Matrix3f J;
    J << parameter_scalar_read(&m_Ixx), parameter_scalar_read(&m_Ixy), parameter_scalar_read(&m_Ixz),
        parameter_scalar_read(&m_Ixy), parameter_scalar_read(&m_Iyy), parameter_scalar_read(&m_Iyz),
        parameter_scalar_read(&m_Ixz), parameter_scalar_read(&m_Iyz), parameter_scalar_read(&m_Izz);
    float m = parameter_scalar_read(&m_mass);

    // angular rate PID control
    status.rate_control = false;
    Eigen::Vector3f angular_acceleration;
    for (int i = 0; i < 3; i++){
        if (isfinite(status.angular_rate_ref[i])) {
            status.rate_control = true;
            float rate_error = att.angular_rate[i] - status.angular_rate_ref[i];
            angular_acceleration[i] = m_pid_controller_rpy[i].process(rate_error);
            angular_acceleration[i] = m_output_lp_rpy[i].process(angular_acceleration[i]);

            angular_acceleration[i] += ctrl_in.angular_acceleration[i];
        } else {
            angular_acceleration[i] = 0;
            m_pid_controller_rpy[i].reset();
        }
    }
    Eigen::Map<Eigen::Vector3f> angular_rate_vec(att.angular_rate);
    Eigen::Map<Eigen::Vector3f> torque_vec(status.torque);
    torque_vec = J * angular_acceleration - (J*angular_rate_vec).cross(angular_rate_vec);

    out.actuators_len = NB_ACTUATORS;
    std::fill(out.actuators.begin(), out.actuators.end(), 0);
    // torque
    std::array<float, NB_ACTUATORS> coeff;
    for (int axis=0; axis < 3; axis++) {
        parameter_vector_read(&m_rpy_ctrl_mix[axis], coeff.data());
        for (int i = 0; i < NB_ACTUATORS; i++) {
            out.actuators[i] += coeff[i] * status.torque[axis];
        }
    }
    // force
    for (int axis=0; axis < 3; axis++) {
        parameter_vector_read(&m_force_ctrl_mix[axis], coeff.data());
        for (int i = 0; i < NB_ACTUATORS; i++) {
            out.actuators[i] += coeff[i] * m * ctrl_in.acceleration[axis];
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



