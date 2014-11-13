#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include <string.h>
#include "usbcfg.h"

#include "sensors/onboardsensors.h"
#include "sensors/ms5611.h"
#include "serial-datagram/serial_datagram.h"
#include "cmp/cmp.h"

BaseSequentialStream* stdout;
SerialUSBDriver SDU1;


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


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
    size_t n, size;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: mem\r\n");
        return;
    }
    n = chHeapStatus(NULL, &size);
    chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
    chprintf(chp, "heap fragments   : %u\r\n", n);
    chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
    static const char *states[] = {CH_STATE_NAMES};
    thread_t *tp;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: threads\r\n");
        return;
    }
    chprintf(chp, "    addr    stack prio refs     state\r\n");
    tp = chRegFirstThread();
    do {
        chprintf(chp, "%08lx %08lx %4lu %4lu %9s\r\n",
                 (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
                 (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
                 states[tp->p_state]);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
}


static void cmd_gyro(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    int i;
    for (i = 0; i < 100; i++) {
        chSysLock();
        int gx = 1000*mpu_gyro_sample.rate[0];
        int gy = 1000*mpu_gyro_sample.rate[1];
        int gz = 1000*mpu_gyro_sample.rate[2];
        chSysUnlock();
        chprintf(chp, "gyro %d %d %d\n", gx, gy, gz);
        chThdSleepMilliseconds(10);
    }
}

static const I2CConfig i2c_cfg = {
    .op_mode = OPMODE_I2C,
    .clock_speed = 400000,
    .duty_cycle = FAST_DUTY_CYCLE_2
};

static void cmd_barometer(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argc;
    (void) argv;
    ms5611_t barometer;

    I2CDriver *driver = &I2CD1;

    i2cStart(driver, &i2c_cfg);
    i2cAcquireBus(driver);

    chprintf(chp, "ms5611 init\r\n");

    int init = ms5611_i2c_init(&barometer, driver, 0);

    if (init != 0) {
        i2cflags_t flags = i2cGetErrors(driver);
        chprintf(chp, "ms5611 init failed: %d, %u\r\n", init, (uint32_t)flags);
        i2cReleaseBus(driver);
        i2cStop(driver);
        return;
    } else {
        chprintf(chp, "ms5611 init succeeded\r\n");
    }

    chThdSleepMilliseconds(100);

    int i = 50;
    while (i-- > 0) {
        uint32_t raw_t, raw_p, press;
        int32_t temp;
        int16_t t;

        t = ms5611_adc_start(&barometer, MS5611_ADC_TEMP, MS5611_OSR_4096);
        if (t < 0) {
            continue;
        }

        chThdSleepMilliseconds((t - 1)/1000 + 1);

        ms5611_adc_read(&barometer, &raw_t);

        t = ms5611_adc_start(&barometer, MS5611_ADC_PRESS, MS5611_OSR_4096);
        if (t < 0) {
            continue;
        }

        chThdSleepMilliseconds((t - 1)/1000 + 1);

        ms5611_adc_read(&barometer, &raw_p);

        press = ms5611_calc_press(&barometer, raw_p, raw_t, &temp);

        chprintf(chp, "pressure: %u, temperature: %u\r\n", press, temp);

        chThdSleepMilliseconds(100);
    }

    i2cReleaseBus(driver);
    i2cStop(driver);
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
  {"gyro", cmd_gyro},
  {"baro", cmd_barometer},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};


static void _stream_imu_values_sndfn(void *arg, const void *p, size_t len)
{
    if (len > 0) {
        chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)p, len);
    }
}

struct buffer_writer
{
    char *buffer;
    uint32_t size;
    uint32_t index;
};

static size_t _stream_imu_values_cmp_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    struct buffer_writer *w = (struct buffer_writer*)ctx->buf;
    if (count <= w->size - w->index) {
        memcpy(w->buffer + w->index, data, count);
        w->index += count;
        return count;
    }
    return 0;
}

// not reentrant!
void stream_imu_values(BaseSequentialStream *out)
{
    static char dtgrm[100];
    static struct buffer_writer writer = {.buffer = dtgrm, .size = sizeof(dtgrm), .index = 0};
    static cmp_ctx_t cmp;
    cmp_init(&cmp, &writer, NULL, _stream_imu_values_cmp_writer);
    while (1) {
        chSysLock();
        float gx = mpu_gyro_sample.rate[0];
        float gy = mpu_gyro_sample.rate[1];
        float gz = mpu_gyro_sample.rate[2];
        float ax = mpu_acc_sample.acceleration[0];
        float ay = mpu_acc_sample.acceleration[1];
        float az = mpu_acc_sample.acceleration[2];
        chSysUnlock();
        writer.index = 0;
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
            serial_datagram_send(dtgrm, writer.index, _stream_imu_values_sndfn, out);
        }
        chThdSleepMilliseconds(10);
    }
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

    // USB Serial Driver
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    onboard_sensors_start();

    stream_imu_values((BaseSequentialStream*)&UART_CONN2);

    shellInit();
    thread_t *shelltp = NULL;
    while (true) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
            }
        } else {
            if (chThdTerminatedX(shelltp)) {
                chThdRelease(shelltp);
                shelltp = NULL;
            }
        }
        chThdSleepMilliseconds(500);

        if (palReadPad(GPIOC, GPIOC_SDCARD_DETECT)) {
            palClearPad(GPIOB, GPIOB_LED_SDCARD);
        } else {
            palSetPad(GPIOB, GPIOB_LED_SDCARD);
        }
    }
}
