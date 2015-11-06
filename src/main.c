#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include <string.h>
#include <usbcfg.h>

#include "git_revision.h"
#include "thread_prio.h"
#include "shell_cmds.h"
#include "sumd_input.h"
#include "onboardsensors.h"
#include "serial-datagram/serial_datagram.h"
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "error.h"
#include "parameter/parameter.h"
#include "sdlog.h"
#include "stream.h"

BaseSequentialStream* stdout;
SerialUSBDriver SDU1;

parameter_namespace_t parameters;


/*
 *  Heartbeat, Error LED thread
 *
 * The Heartbeat-LED double-flashes every second in normal mode and continuously
 *  flashes in safemode. If the error level is above NORMAL, the Heartbeat-LED
 *  is off and the Error-LED indicates the error level.
 * If the error level is WARNING, the Error-LED blinks slowly (once per second)
 * If the error level is CRITICAL, the Error-LED blinks rapidly (10 times per second)
 * If a kernel panic occurs the Error-LED is on and all LEDs are off.
 */
static THD_WORKING_AREA(led_task_wa, 128);
static THD_FUNCTION(led_task, arg)
{
    (void)arg;
    chRegSetThreadName("led_task");
    while (1) {
        int err = error_level_get();
        if (err == ERROR_LEVEL_WARNING) {
            palSetPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(500);
            palClearPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(500);
        } else if (err == ERROR_LEVEL_CRITICAL) {
            palSetPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(50);
            palClearPad(GPIOA, GPIOA_LED_ERROR);
            chThdSleepMilliseconds(50);
        } else {
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
        }
    }
}



#include <ff.h>
static FATFS SDC_FS;
bool fatfs_mounted = false;

void sdcard_mount(void)
{
    if (sdcConnect(&SDCD1)) {
        chprintf(stdout, "SD card: failed to connect\n");
        return;
    }
    FRESULT err;
    err = f_mount(&SDC_FS, "", 0);
    if (err != FR_OK) {
        chprintf(stdout, "SD card: mount failed\n");
        sdcDisconnect(&SDCD1);
        return;
    }
    fatfs_mounted = true;
    palSetPad(GPIOB, GPIOB_LED_SDCARD);
    chprintf(stdout, "SD card mounted\n");
}

void sdcard_unmount(void)
{
    f_mount(NULL, "", 0); // unmount
    palClearPad(GPIOB, GPIOB_LED_SDCARD);
    sdcDisconnect(&SDCD1);
    fatfs_mounted = false;
}

void sdcard_automount(void)
{
    if (palReadPad(GPIOC, GPIOC_SDCARD_DETECT)) {
        if (fatfs_mounted) {
            sdcard_unmount();
        }
    } else {
        if (!fatfs_mounted) {
            sdcard_mount();
        }
    }
}

void file_cat(BaseSequentialStream *out, const char *file_path)
{
    static FIL f;
    FRESULT res = f_open(&f, file_path, FA_READ);
    if (res) {
        chprintf(out, "error %d opening %s\n", res, file_path);
        return;
    }
    static char line[80];
    while (f_gets(line, sizeof(line), &f)) {
        chprintf(out, line);
    }
    f_close(&f);
}


int main(void)
{
    halInit();
    chSysInit();

    chThdCreateStatic(led_task_wa, sizeof(led_task_wa), THD_PRIO_LED, led_task, NULL);

    sdStart(&UART_CONN1, NULL);
    sdStart(&UART_CONN2, NULL);
    sdStart(&UART_CONN3, NULL);
    sdStart(&UART_CONN4, NULL);

    stdout = (BaseSequentialStream*)&UART_CONN1;

    const char *panic_msg = get_panic_message();
    if (panic_msg != NULL) {
        chprintf(stdout, "\n> reboot after panic:\n%s\n", panic_msg);
    } else {
        chprintf(stdout, "\n> boot\n");
    }
    chprintf(stdout, "> version: %s\n", build_git_version);
    if (safemode_active()) {
        chprintf(stdout, "> safemode active\n");
    }

    // USB Serial Driver
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);


    parameter_namespace_declare(&parameters, NULL, NULL); // root namespace
    onboardsensors_declare_parameters(&parameters);

    board_sdcard_pwr_en(true);
    chThdSleepMilliseconds(100);
    sdcStart(&SDCD1, NULL);
    sdcard_mount();
    file_cat(stdout, "/test.txt");

    onboard_sensors_start();

    sumd_input_start((BaseSequentialStream*)&UART_CONN2);

    sdlog_start();
    stream_start((BaseSequentialStream*)&UART_CONN4);

    // while (1) {
    //     chThdSleepMilliseconds(100);
    //     sdcard_automount();
    // }

    shellInit();
    static thread_t *shelltp = NULL;
    static ShellConfig shell_cfg;
    shell_cfg.sc_channel = (BaseSequentialStream*)&SDU1;
    shell_cfg.sc_commands = shell_commands;
    while (true) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                file_cat((BaseSequentialStream*)&SDU1, "/banner.txt");
                shelltp = shellCreate(&shell_cfg, THD_WORKING_AREA_SIZE(2048), THD_PRIO_SHELL);
            }
        } else if (chThdTerminatedX(shelltp)) {
            chThdRelease(shelltp);
            shelltp = NULL;
        }
        chThdSleepMilliseconds(500);
    }
}
