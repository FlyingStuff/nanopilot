#include <math.h>
#include "rate_controller_pid.hpp"

PIDRateController::PIDRateController()
{
}

void PIDRateController::declare_parameters(parameter_namespace_t *ns)
{
    pid_controller_rpy[0].declare_parameters(ns, "pid_roll_controller");
    pid_controller_rpy[1].declare_parameters(ns, "pid_pitch_controller");
    pid_controller_rpy[2].declare_parameters(ns, "pid_yaw_controller");
    output_lp_rpy[0].declare_parameters(ns, "roll_low_pass_cutoff");
    output_lp_rpy[1].declare_parameters(ns, "pitch_low_pass_cutoff");
    output_lp_rpy[2].declare_parameters(ns, "yaw_low_pass_cutoff");
}

void PIDRateController::process(const float rate_setpoint_rpy[3], const float rate_measured_rpy[3], float rate_ctrl_output_rpy[3])
{
    for (int i = 0; i < 3; i++){
        if (isfinite(rate_setpoint_rpy[i])) {
            float rate_error = rate_measured_rpy[i] - rate_setpoint_rpy[i];
            rate_ctrl_output_rpy[i] = pid_controller_rpy[i].process(rate_error);
            rate_ctrl_output_rpy[i] = output_lp_rpy[i].process(rate_ctrl_output_rpy[i]);
        } else {
            rate_ctrl_output_rpy[i] = 0;
            pid_controller_rpy[i].reset();
        }
    }
}

void PIDRateController::set_update_frequency(float freq)
{
    for (int i = 0; i < 3; i++) {
        pid_controller_rpy[i].set_update_frequency(freq);
        output_lp_rpy[i].set_update_frequency(freq);
    }
}

void PIDRateController::reset()
{
    for (int i = 0; i < 3; i++) {
        pid_controller_rpy[i].reset();
    }
}

