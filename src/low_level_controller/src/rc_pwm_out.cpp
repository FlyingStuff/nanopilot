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

