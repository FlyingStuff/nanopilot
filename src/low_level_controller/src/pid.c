#include <stdlib.h>
#include <math.h>
#include "pid.h"


void pid_init(pid_ctrl_t *pid)
{
    pid_set_gains(pid, 1., 0., 0.);
    pid->integrator = 0.;
    pid->previous_error = 0.;
    pid->max_integrator_output = INFINITY;
    pid->frequency = 1.;
}

void pid_set_gains(pid_ctrl_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_get_gains(const pid_ctrl_t *pid, float *kp, float *ki, float *kd)
{
    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;
}

void pid_set_integrator_output_limit(pid_ctrl_t *pid, float max)
{
    pid->max_integrator_output = max;
}

float pid_get_integrator_output_limit(const pid_ctrl_t *pid)
{
    return pid->max_integrator_output;
}

float pid_process(pid_ctrl_t *pid, float error)
{
    if (isnan(error)) {
        return 0;
    }
    float integrator_limit;
    float output;
    pid->integrator += error;

    if (pid->ki > 0){
        integrator_limit = pid->max_integrator_output / pid->ki * pid->frequency;
    } else{
        integrator_limit = 0;
    }

    if (pid->integrator > integrator_limit) {
        pid->integrator = integrator_limit;
    } else if (pid->integrator < -integrator_limit) {
        pid->integrator = -integrator_limit;
    }

    output  = - pid->kp * error;
    output += - pid->ki * pid->integrator / pid->frequency;
    output += - pid->kd * (error - pid->previous_error) * pid->frequency;

    pid->previous_error = error;
    return output;
}

void pid_reset_integral(pid_ctrl_t *pid)
{
    pid->integrator = 0.;
}

void pid_set_frequency(pid_ctrl_t *pid, float frequency)
{
    pid->frequency = frequency;
}

float pid_get_frequency(const pid_ctrl_t *pid)
{
    return pid->frequency;
}
