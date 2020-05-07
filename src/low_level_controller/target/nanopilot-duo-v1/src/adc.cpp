#include "ch.h"
#include "hal.h"
#include "thread_prio.h"
#include "log.h"

void adcerrorcallback(ADCDriver *adcp, adcerror_t err)
{
    (void)adcp;
    (void)err;
    chSysHalt("ADC error");
}

static THD_WORKING_AREA(adc_wa, (1500));
static THD_FUNCTION(adc, arg) {
    (void)arg;
    chRegSetThreadName("adc");
    static ADCConfig adccfg = {};
    adcStart(&ADCD1, &adccfg);

    const int ADC_BUF_DEPTH = 1; // depth of buffer
    const int ADC_CH_NUM = 4;    // number of used ADC channels
    static adcsample_t samples_buf[ADC_BUF_DEPTH * ADC_CH_NUM] = {1, 2, 3, 4}; // results array

    // seq:
    // ADC1 CH4 GPIOA_PIN4_I_BAT_ADC
    // ADC1 CH5 GPIOA_PIN5_V_BAT_ADC
    // ADC1 CH10 GPIOC_PIN0_5V_IN_ADC
    // ADC1 CH13 GPIOC_PIN3_V_SERVO_ADC

    static ADCConversionGroup adcgrp = {
        false, // circular
        (uint16_t)(ADC_CH_NUM), // number of channels
        NULL, // end cb
        adcerrorcallback, // error cb
        // adc_lld_configuration_group_fields F7 is ADCv2/hal_adc_lld.h
        0, // cr1
        ADC_CR2_SWSTART, // cr2
        ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480) +
        ADC_SMPR1_SMP_AN13(ADC_SAMPLE_480), // smpr1
        ADC_SMPR2_SMP_AN4(ADC_SAMPLE_480) +
        ADC_SMPR2_SMP_AN5(ADC_SAMPLE_480), // smpr2
        0, // htr
        0, // ltr
        ADC_SQR1_NUM_CH(ADC_CH_NUM), // sqr1
        0, // sqr2
        ADC_SQR3_SQ1_N(4) +
        ADC_SQR3_SQ2_N(5) +
        ADC_SQR3_SQ3_N(10) +
        ADC_SQR3_SQ4_N(13) // sqr3
    };

    while (1) {
        msg_t ok = adcConvert(&ADCD1, &adcgrp, samples_buf, ADC_BUF_DEPTH);
        if (ok == MSG_OK) {
            log_debug("adc %d       %d", samples_buf[0], samples_buf[2]);
            chThdSleepMilliseconds(1000);
        }
    }
}


void run_adc()
{
    chThdCreateStatic(adc_wa, sizeof(adc_wa), THD_PRIO_ADC, adc, NULL);
}
