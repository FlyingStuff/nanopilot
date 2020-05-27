#include "rc_pwm_out.hpp"
#include "actuators_driver.hpp"
#include "log.h"

static std::array<PWMOutputBank, 6> pwm_banks = { {PWMD3, PWMD12, PWMD4, PWMD9, PWMD13, PWMD14} };
static std::array<PWMActuator, NB_ACTUATORS> actuators;
parameter_namespace_t actuators_namespace;
std::array<parameter_t, 6> pwm_bank_output_period_us_param;


void actuators_init(parameter_namespace_t *ns)
{
    parameter_namespace_declare(&actuators_namespace, ns, "actuators");
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
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[5],
        &actuators_namespace, "pwm_12_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[4],
        &actuators_namespace, "pwm_11_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[3],
        &actuators_namespace, "pwm_9_to_10_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[2],
        &actuators_namespace, "pwm_7_to_8_output_period_us", 20000);
    parameter_scalar_declare_with_default(&pwm_bank_output_period_us_param[1],
        &actuators_namespace, "pwm_5_to_6_output_period_us", 20000);
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

void actuators_set_output(const actuators_t &out)
{
    for(unsigned i = 0; i < pwm_banks.size(); ++i) {
        if (parameter_changed(&pwm_bank_output_period_us_param[i])) {
            pwm_banks[i].set_update_period_us(parameter_scalar_get(&pwm_bank_output_period_us_param[i]));
        }
    }

    static const uint8_t bank_idx_ch_idx_mapping[NB_ACTUATORS][2] = {
        {0, 0},
        {0, 1},
        {0, 2},
        {0, 3},
        {1, 0},
        {1, 1},
        {2, 0},
        {2, 1},
        {3, 0},
        {3, 1},
        {4, 0},
        {5, 0},
    };
    unsigned len = out.actuators_len;
    if (len > NB_ACTUATORS) {
        len = NB_ACTUATORS;
    }

    for (unsigned i = 0; i < NB_ACTUATORS; i++) {
        auto bank_idx = bank_idx_ch_idx_mapping[i][0];
        auto ch_idx = bank_idx_ch_idx_mapping[i][1];
        float ch = 0;
        if (i < len) {
            ch = out.actuators[i];
        }
        auto pwm = actuators[i].get_pulse_width(ch);
        pwm_banks[bank_idx].set_channel_pos_us(ch_idx, pwm);
    }
}

