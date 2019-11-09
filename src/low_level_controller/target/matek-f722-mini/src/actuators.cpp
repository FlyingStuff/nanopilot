#include "rc_pwm_out.hpp"
#include "actuators.hpp"
#include "log.h"

static std::array<PWMOutputBank, 3> pwm_banks = { {PWMD3, PWMD2, PWMD4} };
static std::array<PWMActuator, NB_ACTUATORS> actuators;
parameter_namespace_t actuators_namespace;
std::array<parameter_t, 6> pwm_bank_output_period_us_param;


void initialize_actuators(parameter_namespace_t *ns)
{
    parameter_namespace_declare(&actuators_namespace, ns, "actuators");
    actuators[9].declare_parameters(&actuators_namespace, "pwm10");
    actuators[8].declare_parameters(&actuators_namespace, "pwm9");
    actuators[7].declare_parameters(&actuators_namespace, "pwm8");
    actuators[6].declare_parameters(&actuators_namespace, "pwm7");
    actuators[5].declare_parameters(&actuators_namespace, "pwm6");
    actuators[4].declare_parameters(&actuators_namespace, "pwm5");
    actuators[3].declare_parameters(&actuators_namespace, "pwm4");
    actuators[2].declare_parameters(&actuators_namespace, "pwm3");
    actuators[1].declare_parameters(&actuators_namespace, "pwm2");
    actuators[0].declare_parameters(&actuators_namespace, "pwm1");
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[2],
        &actuators_namespace, "pwm_7_to_8_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[1],
        &actuators_namespace, "pwm_5_6_9_10_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[0],
        &actuators_namespace, "pwm_1_to_4_output_period_us", 20000);

    for(auto& b: pwm_banks) {
        b.initialize();
    }
}

void actuators_disable_all(void)
{
    for(auto& b: pwm_banks) {
        b.disable_all_channels();
    }
}

void actuators_set_output(const std::array<float, NB_ACTUATORS> out)
{
    for(unsigned i = 0; i < pwm_banks.size(); ++i) {
        if (parameter_changed(&pwm_bank_output_period_us_param[i])) {
            pwm_banks[i].set_update_period_us(parameter_scalar_get(&pwm_bank_output_period_us_param[i]));
        }
    }

    pwm_banks[0].set_channel_pos_us(0, actuators[0].get_pulse_width(out[0]));
    pwm_banks[0].set_channel_pos_us(1, actuators[1].get_pulse_width(out[1]));
    pwm_banks[0].set_channel_pos_us(2, actuators[2].get_pulse_width(out[2]));
    pwm_banks[0].set_channel_pos_us(3, actuators[3].get_pulse_width(out[3]));

    pwm_banks[1].set_channel_pos_us(0, actuators[4].get_pulse_width(out[4]));
    pwm_banks[1].set_channel_pos_us(1, actuators[5].get_pulse_width(out[5]));

    pwm_banks[2].set_channel_pos_us(0, actuators[6].get_pulse_width(out[6]));
    pwm_banks[2].set_channel_pos_us(1, actuators[7].get_pulse_width(out[7]));

    pwm_banks[1].set_channel_pos_us(2, actuators[8].get_pulse_width(out[8]));
    pwm_banks[1].set_channel_pos_us(3, actuators[9].get_pulse_width(out[9]));
}

