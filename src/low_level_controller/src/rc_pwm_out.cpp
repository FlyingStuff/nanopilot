#include <ch.h>
#include <hal.h>
#include <cassert>

#include "rc_pwm_out.hpp"

const uint32_t TIMER_FREQUENCY_SERVO = 1000000; // us clock
const uint16_t DEFAULT_UPDATE_PERIOD = 20000;

void PWMOutputBank::initialize()
{
    PWMConfig pwmcfg = {
        TIMER_FREQUENCY_SERVO, // Timer Hz
        DEFAULT_UPDATE_PERIOD,   // period counter PWM
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        },
        0,
        0
    };
    pwmStart(&m_pwm_driver, &pwmcfg);
}

void PWMOutputBank::set_update_period_us(uint16_t period_us)
{
    pwmChangePeriod(&m_pwm_driver, period_us);
}

void PWMOutputBank::set_channel_pos_us(uint8_t channel_idx, uint16_t pulse_width_us)
{
    assert(channel_idx < 4);
    pwmEnableChannel(&m_pwm_driver, channel_idx, pulse_width_us);
}

void PWMOutputBank::disable_channel(uint8_t channel_idx)
{
    pwmDisableChannel(&m_pwm_driver, channel_idx);
}



PWMActuator::PWMActuator()
{
    declare_parameters(NULL, NULL);
}

void PWMActuator::declare_parameters(parameter_namespace_t *parent_ns, const char *name)
{
    parameter_namespace_declare(&m_actuator_ns, parent_ns, name);
    parameter_scalar_declare_with_default(&m_center_pos_us, &m_actuator_ns, "center_pos_us", 1500);
    parameter_scalar_declare_with_default(&m_gain_us, &m_actuator_ns, "gain_us", 500);
    parameter_scalar_declare_with_default(&m_max_pos_us, &m_actuator_ns, "max_pos_us", 2000);
    parameter_scalar_declare_with_default(&m_min_pos_us, &m_actuator_ns, "min_pos_us", 1000);
}


uint16_t PWMActuator::get_pulse_width(float input)
{
    float center = parameter_scalar_read(&m_center_pos_us);
    float gain = parameter_scalar_read(&m_gain_us);
    float max = parameter_scalar_read(&m_max_pos_us);
    float min = parameter_scalar_read(&m_min_pos_us);
    float output = center + input * gain;
    if (output > max) output = max;
    if (output < min) output = min;
    return output;
}
