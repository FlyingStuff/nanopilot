#include "rc_pwm_out.hpp"
#include "actuators.hpp"

std::array<PWMOutputBank, 1> pwm_banks = { {PWMD1} };

void initialize_actuators()
{
    palSetPadMode(GPIOC, 0, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOC, 1, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOC, 2, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOC, 3, PAL_MODE_ALTERNATE(2));

    for(auto& b: pwm_banks) {
        b.initialize();
    }
}

void actuators_set_output(std::array<float, NB_ACTUATORS> out)
{
    for (int i=0; i < 4; i++) {
        pwm_banks[0].set_channel_pos_us(i, out[i]);
    }
}

