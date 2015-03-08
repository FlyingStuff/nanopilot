#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include <string.h>
#include <usbcfg.h>

#include "shell_cmds.h"
#include "sumd_input.h"
#include "sensors/onboardsensors.h"
#include "serial-datagram/serial_datagram.h"
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "error.h"
#include "parameter/parameter.h"

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
    return 0;
}


static void _stream_imu_values_sndfn(void *arg, const void *p, size_t len)
{
    if (len > 0) {
        chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)p, len);
    }
}

// not reentrant!
void stream_imu_values(BaseSequentialStream *out)
{
    static char dtgrm[100];
    static cmp_mem_access_t mem;
    static cmp_ctx_t cmp;
    while (1) {
        chSysLock();
        float gx = mpu_gyro_sample.rate[0];
        float gy = mpu_gyro_sample.rate[1];
        float gz = mpu_gyro_sample.rate[2];
        float ax = mpu_acc_sample.acceleration[0];
        float ay = mpu_acc_sample.acceleration[1];
        float az = mpu_acc_sample.acceleration[2];
        chSysUnlock();
        cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
        bool err = false;
        err = err || !cmp_write_map(&cmp, 2);
        const char *gyro_id = "gyro";
        err = err || !cmp_write_str(&cmp, gyro_id, strlen(gyro_id));
        err = err || !cmp_write_array(&cmp, 3);
        err = err || !cmp_write_float(&cmp, gx);
        err = err || !cmp_write_float(&cmp, gy);
        err = err || !cmp_write_float(&cmp, gz);
        const char *acc_id = "acc";
        err = err || !cmp_write_str(&cmp, acc_id, strlen(acc_id));
        err = err || !cmp_write_array(&cmp, 3);
        err = err || !cmp_write_float(&cmp, ax);
        err = err || !cmp_write_float(&cmp, ay);
        err = err || !cmp_write_float(&cmp, az);
        if (!err) {
            serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
        }
        chThdSleepMilliseconds(10);
    }
}



#include <ff.h>
static FATFS SDC_FS;
bool fatfs_mounted = false;

bool sdcard_mount(void)
{
    if (palReadPad(GPIOC, GPIOC_SDCARD_DETECT)) {
        chprintf(stdout, "no SD card detected\n");
        return false;
    }
    board_sdcard_pwr_en(true);
    chprintf(stdout, "SD card detected\n");
    sdcStart(&SDCD1, NULL);
    if (sdcConnect(&SDCD1)) {
        chprintf(stdout, "SD card: failed to connect\n");
        return false;
    }
    FRESULT err;
    err = f_mount(&SDC_FS, "", 0);
    if (err != FR_OK) {
        chprintf(stdout, "SD card: mount failed\n");
        sdcDisconnect(&SDCD1);
        return false;
    }
    fatfs_mounted = true;
    palSetPad(GPIOB, GPIOB_LED_SDCARD);
    chprintf(stdout, "SD card mounted\n");
    return true;
}

void sdcard_unmount(void)
{
    f_mount(NULL, "", 0); // unmount
    palClearPad(GPIOB, GPIOB_LED_SDCARD);
    sdcDisconnect(&SDCD1);
    fatfs_mounted = false;
}

void file_cat(BaseSequentialStream *out, const char *file_path)
{
    if (palReadPad(GPIOC, GPIOC_SDCARD_DETECT)) {
        sdcard_unmount();
    }
    if (!fatfs_mounted) {
        if (!sdcard_mount()) {
            return;
        }
    }
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

    chThdCreateStatic(led_task_wa, sizeof(led_task_wa), LOWPRIO, led_task, NULL);

    sdStart(&UART_CONN1, NULL);
    sdStart(&UART_CONN2, NULL);
    sdStart(&UART_CONN3, NULL);
    sdStart(&UART_CONN4, NULL);

    stdout = (BaseSequentialStream*)&UART_CONN1;

    chprintf(stdout, "\nboot\n");

    // USB Serial Driver
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);


    parameter_namespace_declare(&parameters, NULL, NULL); // root namespace
    onboardsensors_declare_parameters();

    sdcard_mount();
    file_cat(stdout, "/test.txt");

    onboard_sensors_start();
    // stream_imu_values((BaseSequentialStream*)&UART_CONN2);

    sumd_input_start((BaseSequentialStream*)&UART_CONN2);

    while (SDU1.config->usbp->state != USB_ACTIVE) {
            chThdSleepMilliseconds(100);
    }
    shell_run((BaseSequentialStream*)&SDU1);

    while (true) {
        chThdSleepMilliseconds(500);
    }
}
