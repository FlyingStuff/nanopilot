#ifndef RC_PWM_OUT_HPP
#define RC_PWM_OUT_HPP

#include <hal.h>
#include <cstdint>
#include "parameter/parameter.h"

class PWMOutputBank {
public:
    PWMOutputBank(PWMDriver &pwm_driver) : m_pwm_driver(pwm_driver) {};

    void initialize();

    void set_update_period_us(uint16_t period_us);
    void set_channel_pos_us(uint8_t channel_idx, uint16_t pulse_width_us);
    void disable_channel(uint8_t channel_idx);

private:
    PWMDriver &m_pwm_driver;
};



class PWMActuator {
public:
    PWMActuator();
    void declare_parameters(parameter_namespace_t *parent_ns, const char *name);
    uint16_t get_pulse_width(float input);
private:
    parameter_namespace_t m_actuator_ns;
    parameter_t m_center_pos_us;
    parameter_t m_gain_us;
    parameter_t m_max_pos_us;
    parameter_t m_min_pos_us;
};

#endif /* RC_PWM_OUT_HPP */
