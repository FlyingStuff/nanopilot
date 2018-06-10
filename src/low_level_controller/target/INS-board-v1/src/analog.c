#include <ch.h>
#include <hal.h>
#include "analog.h"


#define ANALOG_VREF_INT_NOMINAL 1.21f // [V] (1.18V min, 1.24V max)
#define ANALOG_RAW_MAX         4095


static void adc_callback(ADCDriver *adcp, adcsample_t *adc_samples, size_t n);
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);

#define DMA_CH_BUFFER_SIZE (128)

// channel conversion sequence
// VREF_INT         : ADC1_IN17
// TEMP_SENSOR      : ADC1_IN16
// V_DC_MON         : ADC123_IN10
// CONN2 TX (PA0)   : ADC123_IN0
// CONN2 RX (PA1)   : ADC123_IN1
// CONN3 TX (PA2)   : ADC123_IN2
// CONN3 RX (PA3)   : ADC123_IN3

static int32_t analog_conversion[ANALOG_NB_CHANNELS];

static adcsample_t adc_dma_buffer[ANALOG_NB_CHANNELS * DMA_CH_BUFFER_SIZE];
static const ADCConversionGroup adcgrpcfg1 = {
    .circular = TRUE,
    .num_channels = ANALOG_NB_CHANNELS,
    .end_cb = adc_callback,
    .error_cb = adcerrorcallback,
    .cr1 = 0,
    .cr2 = ADC_CR2_CONT | ADC_CR2_SWSTART,
    .smpr1 = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480)
           | ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_480)
           | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_480),
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_480)
           | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480)
           | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480)
           | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_480),
    .sqr1 = ADC_SQR1_NUM_CH(ANALOG_NB_CHANNELS),
    .sqr2 = ADC_SQR2_SQ7_N(ADC_CHANNEL_IN3),
    .sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_VREFINT)
          | ADC_SQR3_SQ2_N(ADC_CHANNEL_SENSOR)
          | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN10)
          | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN0)
          | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN1)
          | ADC_SQR3_SQ6_N(ADC_CHANNEL_IN2),
};




float analog_get_raw(unsigned int channel)
{
    chSysLock();
    int32_t adc_val = analog_conversion[channel];
    chSysUnlock();
    return (float)adc_val / (DMA_CH_BUFFER_SIZE / 2);
}


float analog_get_voltage(unsigned int channel)
{
    chSysLock();
    int32_t adc_ref = analog_conversion[ANALOG_CH_VREFINT];
    int32_t adc_val = analog_conversion[channel];
    chSysUnlock();
    return (float)adc_val / adc_ref * ANALOG_VREF_INT_NOMINAL;
}


float analog_get_vcc(void)
{
    float vref_rel = ((float)analog_get_raw(ANALOG_CH_VREFINT) / ANALOG_RAW_MAX);
    return ANALOG_VREF_INT_NOMINAL / vref_rel;
}


float analog_get_vdc(void)
{
    return analog_get_voltage(ANALOG_CH_VDC_MON) * VDC_VOLTAGE_DIV_GAIN;
}


// returns temperature in deg C
float analog_get_cpu_temp(void)
{
    const float volt_at_25_deg = 0.76;
    const float deg_per_volt = 400;
    float temp_volt = analog_get_voltage(ANALOG_CH_TEMP);
    return (temp_volt - volt_at_25_deg) * deg_per_volt + 25;
}


static void adc_callback(ADCDriver *adcp, adcsample_t *adc_samples, size_t n)
{
    (void)adcp;
    osalDbgAssert(n == DMA_CH_BUFFER_SIZE/2, "unexpected adc buffer size");

    int ch;
    for (ch = 0; ch < ANALOG_NB_CHANNELS; ch++) {
        analog_conversion[ch] = 0;
        int i;
        for (i = 0; i < (int)n; i++) {
            analog_conversion[ch] += adc_samples[i * ANALOG_NB_CHANNELS + ch];
        }
    }
}


static void adcerrorcallback(ADCDriver *adcp, adcerror_t err)
{
    (void)adcp;
    if (err == ADC_ERR_DMAFAILURE) {
        chSysHalt("ADC_ERR_DMAFAILURE");
    }
    if (err == ADC_ERR_OVERFLOW) {
        chSysHalt("ADC_ERR_OVERFLOW");
    }
    chSysHalt("ADC error");
}

void analog_configure_conn_3(void)
{
    // uart conn 3 rx pin (PA3)
    // palSetPadMode(GPIOA, GPIOA_UART2_RX_CONN3, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, GPIOA_UART2_TX_CONN3, PAL_MODE_INPUT_ANALOG);
}


static THD_FUNCTION(adc_task, arg)
{
    (void)arg;
    chRegSetThreadName("adc read");

    adcStart(&ADCD1, NULL);

    adcConvert(&ADCD1, &adcgrpcfg1, adc_dma_buffer, DMA_CH_BUFFER_SIZE); // should never return
}

void analog_start(void)
{
    static THD_WORKING_AREA(adc_task_wa, 128);
    chThdCreateStatic(adc_task_wa, sizeof(adc_task_wa), HIGHPRIO, adc_task, NULL);

    chThdSleepMilliseconds(100); // todo
    adcSTM32EnableTSVREFE(); // enable temperature sensor & voltage reference (the adc must be running)
}
