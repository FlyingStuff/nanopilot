#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <shell.h>

#include "sensors/onboardsensors.h"
#include "sensors/ms5611.h"
#include "serial-datagram/serial_datagram.h"

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

static ShellConfig shell_cfg;
static THD_WORKING_AREA(shell_wa, 256);

void shell_run(BaseSequentialStream* dev)
{
    static bool shell_running = false;
    if (!shell_running) {
        shell_running = true;
        shellInit();
        shell_cfg.sc_channel = dev;
        shell_cfg.sc_commands = commands;
        shellCreateStatic(&shell_cfg, shell_wa, sizeof(shell_wa), NORMALPRIO);
    }
}
