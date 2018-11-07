#ifndef RC_PWM_OUT_HPP
#define RC_PWM_OUT_HPP

#include <hal.h>
#include <cstdint>

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

#endif /* RC_PWM_OUT_HPP */
