#include <ch.h>
#include <hal.h>
#include "exti.h"

event_source_t exti_events;

static void gpio_exti_callback(EXTDriver *extp, expchannel_t channel) {
    if (channel == GPIOB_HMC5883L_DRDY) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_HMC5883L_DRDY);
        chSysUnlockFromISR();
    } else if (channel == GPIOC_MPU6000_INT) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_MPU6000_INT);
        chSysUnlockFromISR();
    } else if (channel == GPIOC_H3LIS331DL_INT) {
        chSysLockFromISR();
        chEvtBroadcastFlagsI(&exti_events, EXTI_EVENT_H3LIS331DL_INT);
        chSysUnlockFromISR();
    }
}

static const EXTConfig extcfg = {{
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, gpio_exti_callback}, // GPIOB_HMC5883L_DRDY
    {EXT_CH_MODE_DISABLED, NULL}, // GPIOC_SDCARD_DETECT
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, gpio_exti_callback}, // GPIOC_MPU6000_INT
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOC, gpio_exti_callback}, // GPIOC_H3LIS331DL_INT
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
}};

void exti_setup(void)
{
    chEvtObjectInit(&exti_events);
    extStart(&EXTD1, &extcfg);
}
