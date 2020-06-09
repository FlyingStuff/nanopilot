#ifndef ATTITUDE_CONTROLLER_HPP
#define ATTITUDE_CONTROLLER_HPP

#include "control_loop.hpp"
#include "sensors.hpp"
#include "attitude_filter.hpp"
#include "pid_with_parameter.hpp"
#include "low_pass_filter.hpp"


typedef struct {
    struct quaternion_s attitude; // can be NaN to disable attitude ctrl
    float angular_rate[3]; // individual axes can be NaN to disable rate ctrl
    float angular_acceleration[3];
    float acceleration[3];
    timestamp_t timestamp;
} attitude_controller_input_t;


typedef struct {
    float angular_rate_ref[3];
    float torque[3];
    struct quaternion_s attitude_setpt;
    bool attitude_control;
    bool rate_control;
    timestamp_t timestamp;
} attitude_controller_status_t;

extern msgbus::Topic<attitude_controller_input_t> attitude_controller_input_topic;
extern msgbus::Topic<attitude_controller_status_t> attitude_controller_status_topic;


class AttitudeController: public ControllerInterface {
private:
    PIDController m_pid_controller_rpy[3];
    LowPassFilter m_output_lp_rpy[3];
    parameter_namespace_t m_namespace;
    parameter_t m_attitude_roll_time_cst;
    parameter_t m_attitude_pitch_time_cst;
    parameter_t m_attitude_yaw_time_cst;
    parameter_t m_attitude_tilt_twist_decompostition;
    parameter_t m_max_roll_rate;
    parameter_t m_max_pitch_rate;
    parameter_t m_max_yaw_rate;
    parameter_t m_Ixx;
    parameter_t m_Iyy;
    parameter_t m_Izz;
    parameter_t m_Ixy;
    parameter_t m_Ixz;
    parameter_t m_Iyz;
    parameter_t m_mass;
    parameter_t m_rc_roll_rate_gain;
    parameter_t m_rc_pitch_rate_gain;
    parameter_t m_rc_yaw_rate_gain;
    parameter_t m_rc_acc_x_gain;
    parameter_t m_rc_acc_y_gain;
    parameter_t m_rc_acc_z_gain;
    float m_rpy_ctrl_mix_buf[3][NB_ACTUATORS] = {{0}};
    parameter_t m_rpy_ctrl_mix[3];
    float m_force_ctrl_mix_buf[3][NB_ACTUATORS] = {{0}};
    parameter_t m_force_ctrl_mix[3];
    msgbus::Subscriber<msgbus::Topic<attitude_controller_input_t>> m_ctrl_in_sub;
    msgbus::Subscriber<msgbus::Topic<attitude_filter_output_t>> m_att_sub;

public:
    explicit AttitudeController();
    void declare_parameters(parameter_namespace_t *ns);

    virtual control_mode_t process(const rc_input_s &rc_in, actuators_t &out);
    virtual void notify_output_disabled();
    virtual void set_update_frequency(float freq);
    virtual timestamp_t ap_control_signal_timestamp();
};


#endif /* ATTITUDE_CONTROLLER_HPP */
