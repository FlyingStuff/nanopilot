#include "rate_controller_pid.hpp"

PIDRateController::PIDRateController()
{
}

void PIDRateController::declare_parameters(parameter_namespace_t *ns)
{
    pid_roll_controller.declare_parameters(ns, "pid_roll_controller");
    pid_pitch_controller.declare_parameters(ns, "pid_pitch_controller");
    pid_yaw_controller.declare_parameters(ns, "pid_yaw_controller");
    roll_lp.declare_parameters(ns, "roll_low_pass_cutoff");
    pitch_lp.declare_parameters(ns, "pitch_low_pass_cutoff");
    yaw_lp.declare_parameters(ns, "yaw_low_pass_cutoff");
}

void PIDRateController::process(const float rate_setpoint_rpy[3], const float rate_measured_rpy[3], float rate_ctrl_output_rpy[3])
{
    float error_rate_rpy[3];
    int i;
    for (i = 0; i < 3; i++){
        error_rate_rpy[i] = rate_measured_rpy[i] - rate_setpoint_rpy[i];
    }
    rate_ctrl_output_rpy[0] = pid_roll_controller.process(error_rate_rpy[0]);
    rate_ctrl_output_rpy[1] = pid_pitch_controller.process(error_rate_rpy[1]);
    rate_ctrl_output_rpy[2] = pid_yaw_controller.process(error_rate_rpy[2]);
    rate_ctrl_output_rpy[0] = roll_lp.process(rate_ctrl_output_rpy[0]);
    rate_ctrl_output_rpy[1] = pitch_lp.process(rate_ctrl_output_rpy[1]);
    rate_ctrl_output_rpy[2] = yaw_lp.process(rate_ctrl_output_rpy[2]);
}

void PIDRateController::set_update_frequency(float freq)
{
    pid_roll_controller.set_update_frequency(freq);
    pid_pitch_controller.set_update_frequency(freq);
    pid_yaw_controller.set_update_frequency(freq);
    roll_lp.set_update_frequency(freq);
    pitch_lp.set_update_frequency(freq);
    yaw_lp.set_update_frequency(freq);
}

void PIDRateController::reset()
{
    pid_roll_controller.reset();
    pid_pitch_controller.reset();
    pid_yaw_controller.reset();
}

