#include <ch.h>
#include <hal.h>
#include "thread_prio.h"
#include <fatfs_diskio.h>
#include "led.h"
#include "sdcard.h"
#include "error.h"


static void sd_card_activity_led_trigger(void)
{
    chSysLock();
    palTogglePad(GPIOB, GPIOB_LED_SDCARD);
    chSysUnlock();
}

static void sd_card_activity_led_periodic_reset(void)
{
    if (sdcard_is_mounted()) {
        palSetPad(GPIOB, GPIOB_LED_SDCARD);
    } else {
        palClearPad(GPIOB, GPIOB_LED_SDCARD);
    }
}

/*
 *  Heartbeat LED thread
 *
 * The Heartbeat-LED double-flashes every second in normal mode and continuously
 *  flashes in safemode.
 */
static THD_WORKING_AREA(led_task_wa, 128);
static THD_FUNCTION(led_task, arg)
{
    (void)arg;
    chRegSetThreadName("led_task");
    while (1) {
        palSetPad(GPIOA, GPIOA_LED_HEARTBEAT);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
        chThdSleepMilliseconds(80);
        palSetPad(GPIOA, GPIOA_LED_HEARTBEAT);
        chThdSleepMilliseconds(80);
        palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
        if (safemode_active()) {
            chThdSleepMilliseconds(80);
        } else {
            chThdSleepMilliseconds(760);
        }
        sd_card_activity_led_periodic_reset();
    }
}

void led_start(void)
{
    fatfs_diskio_set_io_callback(sd_card_activity_led_trigger);
    chThdCreateStatic(led_task_wa, sizeof(led_task_wa), THD_PRIO_LED, led_task, NULL);
}

