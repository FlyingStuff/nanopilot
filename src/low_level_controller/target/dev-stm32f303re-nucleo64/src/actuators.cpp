#include "rc_pwm_out.hpp"
#include "actuators.hpp"
#include "log.h"

static std::array<PWMOutputBank, 1> pwm_banks = { {PWMD1} };
static std::array<PWMActuator, NB_ACTUATORS> actuators;

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

    for(auto& b: pwm_banks) {
        b.initialize();
    }
}

void actuators_set_output(std::array<float, NB_ACTUATORS> out)
{
    for (int i=0; i < 4; i++) {
        uint16_t pulse_us = actuators[i].get_pulse_width(out[i]);
        pwm_banks[0].set_channel_pos_us(i, pulse_us);
    }
}

