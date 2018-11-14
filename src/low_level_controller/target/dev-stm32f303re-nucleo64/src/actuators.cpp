#include "rc_pwm_out.hpp"
#include "actuators.hpp"
#include "log.h"

static std::array<PWMOutputBank, 1> pwm_banks = { {PWMD1} };
static std::array<PWMActuator, NB_ACTUATORS> actuators;
parameter_t pwm_bank_1_output_period_us_param;

void initialize_actuators(parameter_namespace_t *ns)
{
    palSetPadMode(GPIOC, 0, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOC, 1, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOC, 2, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOC, 3, PAL_MODE_ALTERNATE(2));

    actuators[3].declare_parameters(ns, "pwm4");
    actuators[2].declare_parameters(ns, "pwm3");
    actuators[1].declare_parameters(ns, "pwm2");
    actuators[0].declare_parameters(ns, "pwm1");
    parameter_scalar_declare_with_default(&pwm_bank_1_output_period_us_param,
        ns, "pwm_1_to_4_output_period_us", 20000);

    for(auto& b: pwm_banks) {
        b.initialize();
    }
}

void actuators_disable_all(void)
{
    for(auto& b: pwm_banks) {
        b.disable_channel(0);
        b.disable_channel(1);
        b.disable_channel(2);
        b.disable_channel(3);
    }
}

void actuators_set_output(const std::array<float, NB_ACTUATORS> out)
{
    if (parameter_changed(&pwm_bank_1_output_period_us_param)) {
        pwm_banks[0].set_update_period_us(parameter_scalar_get(&pwm_bank_1_output_period_us_param));
    }
    for (int i=0; i < 4; i++) {
        uint16_t pulse_us = actuators[i].get_pulse_width(out[i]);
        pwm_banks[0].set_channel_pos_us(i, pulse_us);
    }
}

