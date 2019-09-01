#include <stdlib.h>
#include <math.h>
#include "pid/pid.hpp"


void PID::set_gains(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

PID::PID()
{
    this->set_gains(1., 0., 0.);
    this->integrator = 0.;
    this->previous_error = 0.;
    this->max_integrator_output = INFINITY;
    this->frequency = 1.;
}

void PID::get_gains(double &kp, double &ki, double &kd)
{
    kp = this->kp;
    ki = this->ki;
    kd = this->kd;
}

void PID::set_integrator_output_limit(double max)
{
    this->max_integrator_output = max;
}

double PID::get_integrator_output_limit()
{
    return this->max_integrator_output;
}

double PID::process(double error)
{
    if (isnan(error)) {
        return 0;
    }
    double integrator_limit;
    double output;
    this->integrator += error;

    if (this->ki > 0){
        integrator_limit = this->max_integrator_output / this->ki * this->frequency;
    } else{
        integrator_limit = 0;
    }

    if (this->integrator > integrator_limit) {
        this->integrator = integrator_limit;
    } else if (this->integrator < -integrator_limit) {
        this->integrator = -integrator_limit;
    }

    output  = - this->kp * error;
    output += - this->ki * this->integrator / this->frequency;
    output += - this->kd * (error - this->previous_error) * this->frequency;

    this->previous_error = error;
    return output;
}

void PID::reset_integral()
{
    this->integrator = 0.;
}

void PID::set_frequency(double frequency)
{
    this->frequency = frequency;
}

double PID::get_frequency()
{
    return this->frequency;
}

void PID::set_kp(double kp)
{
    this->kp = kp;
}

void PID::set_ki(double ki)
{
    this->ki = ki;
}

void PID::set_kd(double kd)
{
    this->kd = kd;
}

PID::~PID()
{
}
