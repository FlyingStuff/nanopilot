#include "rc_pwm_out.hpp"
#include "actuators.hpp"
#include "log.h"

static std::array<PWMOutputBank, 4> pwm_banks = { {PWMD1, PWMD8, PWMD4, PWMD3} };
static std::array<PWMActuator, NB_ACTUATORS> actuators;
parameter_namespace_t actuators_namespace;
parameter_t pwm_bank_1_output_period_us_param;
parameter_t pwm_bank_2_output_period_us_param;
parameter_t pwm_bank_3_output_period_us_param;
parameter_t pwm_bank_4_output_period_us_param;

void initialize_actuators(parameter_namespace_t *ns)
{
    parameter_namespace_declare(&actuators_namespace, ns, "actuators");
    actuators[15].declare_parameters(&actuators_namespace, "pwm16");
    actuators[14].declare_parameters(&actuators_namespace, "pwm15");
    actuators[13].declare_parameters(&actuators_namespace, "pwm14");
    actuators[12].declare_parameters(&actuators_namespace, "pwm13");
    actuators[11].declare_parameters(&actuators_namespace, "pwm12");
    actuators[10].declare_parameters(&actuators_namespace, "pwm11");
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
    parameter_scalar_declare_with_default(&pwm_bank_4_output_period_us_param,
        &actuators_namespace, "pwm_13_to_16_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_3_output_period_us_param,
        &actuators_namespace, "pwm_9_to_12_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_2_output_period_us_param,
        &actuators_namespace, "pwm_5_to_8_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_1_output_period_us_param,
        &actuators_namespace, "pwm_1_to_4_output_period_us", 20000);


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
    if (parameter_changed(&pwm_bank_2_output_period_us_param)) {
        pwm_banks[1].set_update_period_us(parameter_scalar_get(&pwm_bank_2_output_period_us_param));
    }
    if (parameter_changed(&pwm_bank_3_output_period_us_param)) {
        pwm_banks[2].set_update_period_us(parameter_scalar_get(&pwm_bank_3_output_period_us_param));
    }
    if (parameter_changed(&pwm_bank_4_output_period_us_param)) {
        pwm_banks[3].set_update_period_us(parameter_scalar_get(&pwm_bank_4_output_period_us_param));
    }

    pwm_banks[0].set_channel_pos_us(0, actuators[0].get_pulse_width(out[0]));
    pwm_banks[0].set_channel_pos_us(1, actuators[1].get_pulse_width(out[1]));
    pwm_banks[0].set_channel_pos_us(2, actuators[2].get_pulse_width(out[2]));
    pwm_banks[0].set_channel_pos_us(3, actuators[3].get_pulse_width(out[3]));

    pwm_banks[1].set_channel_pos_us(3, actuators[4].get_pulse_width(out[4]));
    pwm_banks[1].set_channel_pos_us(2, actuators[5].get_pulse_width(out[5]));
    pwm_banks[1].set_channel_pos_us(1, actuators[6].get_pulse_width(out[6]));
    pwm_banks[1].set_channel_pos_us(0, actuators[7].get_pulse_width(out[7]));

    pwm_banks[2].set_channel_pos_us(0, actuators[8].get_pulse_width(out[8]));
    pwm_banks[2].set_channel_pos_us(1, actuators[9].get_pulse_width(out[9]));
    pwm_banks[2].set_channel_pos_us(2, actuators[10].get_pulse_width(out[10]));
    pwm_banks[2].set_channel_pos_us(3, actuators[11].get_pulse_width(out[11]));

    pwm_banks[3].set_channel_pos_us(0, actuators[12].get_pulse_width(out[12]));
    pwm_banks[3].set_channel_pos_us(1, actuators[13].get_pulse_width(out[13]));
    pwm_banks[3].set_channel_pos_us(2, actuators[14].get_pulse_width(out[14]));
    pwm_banks[3].set_channel_pos_us(3, actuators[15].get_pulse_width(out[15]));
}

