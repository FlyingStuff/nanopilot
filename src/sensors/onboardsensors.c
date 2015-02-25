#include <math.h>
#include <ch.h>
#include "mpu60X0.h"
#include "hmc5883l.h"
#include "ms5611.h"
#include "exti.h"
#include "parameter/parameter.h"
#include "main.h"

#include "imu.h"

rate_gyro_sample_t mpu_gyro_sample;
accelerometer_sample_t mpu_acc_sample;

#define SENSOR_INTERRUPT_EVENT 1


static parameter_namespace_t sensor_param;
static parameter_namespace_t mpu6000_param;
static parameter_t mpu6000_gyro_full_scale;
static parameter_t mpu6000_acc_full_scale;


void onboardsensors_declare_parameters(void)
{
    parameter_namespace_declare(&sensor_param, &parameters, "sensors");
    parameter_namespace_declare(&mpu6000_param, &sensor_param, "mpu6000");
    parameter_scalar_declare_with_default(&mpu6000_gyro_full_scale, &mpu6000_param, "gyro_full_scale", 500); // [deg/s]
    parameter_scalar_declare_with_default(&mpu6000_acc_full_scale, &mpu6000_param, "acc_full_scale", 2); // [g]
}

static int mpu6000_init(mpu60X0_t *dev, rate_gyro_t *gyro, accelerometer_t *acc)
{
    /*
     * SPI1 configuration structure for MPU6000.
     * SPI1 is on APB2 @ 84MHz / 128 = 656.25kHz
     * CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
     */
    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOC,
        .sspad = GPIOC_MPU6000_CS,
        .cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
    };

    spiStart(&SPID1, &spi_cfg);
    if (!mpu60X0_ping(dev)) {
        return -1;
    }

    int config = MPU60X0_LOW_PASS_FILTER_6 | MPU60X0_SAMPLE_RATE_DIV(0);
    float afs = parameter_scalar_get(&mpu6000_acc_full_scale);
    float gfs = parameter_scalar_get(&mpu6000_gyro_full_scale);
    float afs_mps, gfs_radps;
    if (afs <= 2) {
        afs_mps = 2*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_2G;
    } else if (afs <= 4) {
        afs_mps = 4*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_4G;
    } else if (afs <= 8) {
        afs_mps = 8*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_8G;
    } else {
        afs_mps = 16*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_16G;
    }
    if (gfs <= 250) {
        gfs_radps = 250*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_250DPS;
    } else if (gfs <= 500) {
        gfs_radps = 500*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_500DPS;
    } else if (gfs <= 1000) {
        gfs_radps = 1000*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_1000DPS;
    } else {
        gfs_radps = 2000*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_2000DPS;
    }
    acc->full_scale_range[0] = afs_mps;
    acc->full_scale_range[1] = afs_mps;
    acc->full_scale_range[2] = afs_mps;
    gyro->full_scale_range[0] = gfs_radps;
    gyro->full_scale_range[1] = gfs_radps;
    gyro->full_scale_range[2] = gfs_radps;

    mpu60X0_setup(dev, config);

    /* speed up SPI for sensor register reads (max 20MHz)
     * APB2 @ 84MHz / 8 = 10.5MHz
     */
    spi_cfg.cr1 = SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA;
    spiStart(&SPID1, &spi_cfg);
    // check that the sensor still pings
    if (!mpu60X0_ping(dev)) {
        return -1;
    }
    return 0;
}


static THD_WORKING_AREA(spi_sensors_wa, 128);
static THD_FUNCTION(spi_sensors, arg)
{
    (void)arg;
    chRegSetThreadName("onboard-sensors-spi");
    static event_listener_t sensor_int;
    chEvtRegisterMaskWithFlags(&exti_events, &sensor_int,
                               (eventmask_t)SENSOR_INTERRUPT_EVENT,
                               (eventflags_t)EXTI_EVENT_MPU6000_INT);

    static mpu60X0_t mpu6000;
    mpu60X0_init_using_spi(&mpu6000, &SPID1);

    static rate_gyro_t mpu_gyro = {
        .device = "MPU6000", .noise_stddev = {NAN, NAN, NAN},
        .update_rate = 1000,.health = SENSOR_HEALTH_OK };
    static accelerometer_t mpu_acc = {
        .device = "MPU6000", .noise_stddev = {NAN, NAN, NAN},
        .update_rate = 1000, .health = SENSOR_HEALTH_OK };

    mpu_gyro_sample.sensor = &mpu_gyro;
    mpu_acc_sample.sensor = &mpu_acc;

    if (mpu6000_init(&mpu6000, &mpu_gyro, &mpu_acc) != 0) {
        error_set(ERROR_LEVEL_CRITICAL);
        return 0;
    }

    while (1) {
        // timestamp_t timestamp;
        float gyro[3], acc[3], temp;
        chEvtWaitAny(SENSOR_INTERRUPT_EVENT);
        mpu60X0_read(&mpu6000, gyro, acc, &temp);
        chSysLock();
        mpu_gyro_sample.rate[0] = gyro[0];
        mpu_gyro_sample.rate[1] = gyro[1];
        mpu_gyro_sample.rate[2] = gyro[2];
        mpu_acc_sample.acceleration[0] = acc[0];
        mpu_acc_sample.acceleration[1] = acc[1];
        mpu_acc_sample.acceleration[2] = acc[2];
        chSysUnlock();
    }
    return 0;
}


static THD_WORKING_AREA(i2c_sensors_wa, 256);
static THD_FUNCTION(i2c_sensors, arg)
{
    (void)arg;
    static const I2CConfig i2c_cfg = {
        .op_mode = OPMODE_I2C,
        .clock_speed = 400000,
        .duty_cycle = FAST_DUTY_CYCLE_2
    };
    I2CDriver *driver = &I2CD1;

    i2cStart(driver, &i2c_cfg);

    static ms5611_t barometer;

    i2cAcquireBus(driver);
    int init = ms5611_i2c_init(&barometer, driver, 0);
    i2cReleaseBus(driver);
    if (init != 0) {
        // i2cflags_t flags = i2cGetErrors(driver);
        // chprintf(out, "ms5611 init failed: %d, %u\r\n", init, (uint32_t)flags);
        error_set(ERROR_LEVEL_WARNING);
    }

    while (1) {
        /*
         *  Barometer
         */
        uint32_t raw_t, raw_p, press;
        int32_t temp, t;
        i2cAcquireBus(driver);
        t = ms5611_adc_start(&barometer, MS5611_ADC_TEMP, MS5611_OSR_4096);
        i2cReleaseBus(driver);
        if (t > 0) {
            chThdSleepMilliseconds((t - 1)/1000 + 1);
            i2cAcquireBus(driver);
            ms5611_adc_read(&barometer, &raw_t);
            i2cReleaseBus(driver);
        }

        i2cAcquireBus(driver);
        t = ms5611_adc_start(&barometer, MS5611_ADC_PRESS, MS5611_OSR_4096);
        i2cReleaseBus(driver);
        if (t > 0) {
            chThdSleepMilliseconds((t - 1)/1000 + 1);
            i2cAcquireBus(driver);
            ms5611_adc_read(&barometer, &raw_p);
            i2cReleaseBus(driver);
        }
        press = ms5611_calc_press(&barometer, raw_p, raw_t, &temp);


        chThdSleepMilliseconds(100);
    }
    return 0;
}


void onboard_sensors_start(void)
{
    exti_setup();
    chThdCreateStatic(spi_sensors_wa, sizeof(spi_sensors_wa), LOWPRIO, spi_sensors, NULL);
    chThdCreateStatic(i2c_sensors_wa, sizeof(i2c_sensors_wa), LOWPRIO, i2c_sensors, NULL);
}
