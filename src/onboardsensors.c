#include <math.h>
#include <ch.h>
#include "thread_prio.h"
#include "log.h"
#include "sensors/mpu60X0.h"
#include "sensors/hmc5883l.h"
#include "sensors/ms5611.h"
#include "sensors/h3lis331dl.h"
#include "exti.h"
#include "parameter/parameter.h"
#include "sensors.h"
#include "timestamp/timestamp.h"

#include "onboardsensors.h"

static rate_gyro_sample_t onboard_mpu6000_gyro_sample;
static accelerometer_sample_t onboard_mpu6000_acc_sample;
static float onboard_mpu6000_temp;
static accelerometer_sample_t onboard_h3lis331dl_acc_sample;
static magnetometer_sample_t onboard_hmc5883l_mag_sample;
static barometer_sample_t onboard_ms5511_baro_sample;

event_source_t sensor_events;


#define EXTI_INTERRUPT_EVENT    1

static parameter_namespace_t sensor_param;
static parameter_namespace_t mpu6000_param;
static parameter_t mpu6000_gyro_full_scale;
static parameter_t mpu6000_acc_full_scale;


void onboard_sensor_get_mpu6000_gyro_sample(rate_gyro_sample_t *out)

{
    chSysLock();
    *out = onboard_mpu6000_gyro_sample;
    chSysUnlock();
}

void onboard_sensor_get_mpu6000_acc_sample(accelerometer_sample_t *out)
{
    chSysLock();
    *out = onboard_mpu6000_acc_sample;
    chSysUnlock();
}

float onboard_sensor_get_mpu6000_temp(void)
{
    chSysLock();
    float out = onboard_mpu6000_temp;
    chSysUnlock();
    return out;
}

void onboard_sensor_get_h3lis331dl_acc_sample(accelerometer_sample_t *out)
{
    chSysLock();
    *out = onboard_h3lis331dl_acc_sample;
    chSysUnlock();
}

void onboard_sensor_get_hmc5883l_mag_sample(magnetometer_sample_t *out)
{
    chSysLock();
    *out = onboard_hmc5883l_mag_sample;
    chSysUnlock();
}

void onboard_sensor_get_ms5511_baro_sample(barometer_sample_t *out)
{
    chSysLock();
    *out = onboard_ms5511_baro_sample;
    chSysUnlock();
}



void onboardsensors_declare_parameters(parameter_namespace_t *namespace)
{
    parameter_namespace_declare(&sensor_param, namespace, "sensors");
    parameter_namespace_declare(&mpu6000_param, &sensor_param, "mpu6000");
    parameter_scalar_declare_with_default(&mpu6000_gyro_full_scale, &mpu6000_param, "gyro_full_scale", 2000); // [deg/s]
    parameter_scalar_declare_with_default(&mpu6000_acc_full_scale, &mpu6000_param, "acc_full_scale", 16); // [g]
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
    if (afs <= 2) {
        acc->full_scale_range = 2*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_2G;
    } else if (afs <= 4) {
        acc->full_scale_range = 4*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_4G;
    } else if (afs <= 8) {
        acc->full_scale_range = 8*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_8G;
    } else {
        acc->full_scale_range = 16*9.81;
        config |= MPU60X0_ACC_FULL_RANGE_16G;
    }
    if (gfs <= 250) {
        gyro->full_scale_range = 250*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_250DPS;
    } else if (gfs <= 500) {
        gyro->full_scale_range = 500*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_500DPS;
    } else if (gfs <= 1000) {
        gyro->full_scale_range = 1000*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_1000DPS;
    } else {
        gyro->full_scale_range = 2000*M_PI/360;
        config |= MPU60X0_GYRO_FULL_RANGE_2000DPS;
    }

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


static THD_WORKING_AREA(spi_sensors_wa, 256);
static THD_FUNCTION(spi_sensors, arg)
{
    (void)arg;
    chRegSetThreadName("onboard-sensors-spi");
    static event_listener_t sensor_int;
    chEvtRegisterMaskWithFlags(&exti_events, &sensor_int,
                               (eventmask_t)EXTI_INTERRUPT_EVENT,
                               (eventflags_t)EXTI_EVENT_MPU6000_INT);

    static mpu60X0_t mpu6000;
    mpu60X0_init_using_spi(&mpu6000, &SPID1);

    static rate_gyro_t mpu_gyro = { .device = "MPU6000", .update_rate = 1000};
    static accelerometer_t mpu_acc = { .device = "MPU6000", .update_rate = 1000};

    onboard_mpu6000_gyro_sample.sensor = &mpu_gyro;
    onboard_mpu6000_acc_sample.sensor = &mpu_acc;

    if (mpu6000_init(&mpu6000, &mpu_gyro, &mpu_acc) != 0) {
        log_error("mpu6000 init failed");
        return;
    }

    while (1) {
        float gyro[3], acc[3], temp;
        chEvtWaitAny(EXTI_INTERRUPT_EVENT);
        chEvtGetAndClearFlags(&sensor_int);
        timestamp_t t = timestamp_get();
        mpu60X0_read(&mpu6000, gyro, acc, &temp);
        chSysLock();
        // the mpu6000 is mounted with an offset of -90deg around z
        onboard_mpu6000_gyro_sample.rate[0] = gyro[1];
        onboard_mpu6000_gyro_sample.rate[1] = -gyro[0];
        onboard_mpu6000_gyro_sample.rate[2] = gyro[2];
        onboard_mpu6000_gyro_sample.timestamp = t;
        onboard_mpu6000_acc_sample.acceleration[0] = acc[1];
        onboard_mpu6000_acc_sample.acceleration[1] = -acc[0];
        onboard_mpu6000_acc_sample.acceleration[2] = acc[2];
        onboard_mpu6000_acc_sample.timestamp = t;
        onboard_mpu6000_temp = temp;
        chSysUnlock();
        chEvtBroadcastFlags(&sensor_events, SENSOR_EVENT_MPU6000);
    }
}



static THD_WORKING_AREA(i2c_barometer_wa, 256);
static THD_FUNCTION(i2c_barometer, arg)
{
    chRegSetThreadName("onboard-sensors-i2c-baro");
    I2CDriver *i2c_driver = &I2CD1;
    ms5611_t *barometer = (ms5611_t*)arg;

    while (1) {
        uint32_t raw_t, raw_p, press;
        int32_t temp, t;
        i2cAcquireBus(i2c_driver);
        t = ms5611_adc_start(barometer, MS5611_ADC_TEMP, MS5611_OSR_4096);
        i2cReleaseBus(i2c_driver);
        if (t > 0) {
            chThdSleepMilliseconds((t - 1)/1000 + 1);
            i2cAcquireBus(i2c_driver);
            ms5611_adc_read(barometer, &raw_t);
            i2cReleaseBus(i2c_driver);
        }

        i2cAcquireBus(i2c_driver);
        t = ms5611_adc_start(barometer, MS5611_ADC_PRESS, MS5611_OSR_4096);
        i2cReleaseBus(i2c_driver);
        timestamp_t timestamp = timestamp_get();

        if (t > 0) {
            chThdSleepMilliseconds((t - 1)/1000 + 1);
            i2cAcquireBus(i2c_driver);
            ms5611_adc_read(barometer, &raw_p);
            i2cReleaseBus(i2c_driver);
        }
        press = ms5611_calc_press(barometer, raw_p, raw_t, &temp);

        chSysLock();
        onboard_ms5511_baro_sample.pressure = press;
        onboard_ms5511_baro_sample.temperature = (float)temp/100;
        onboard_ms5511_baro_sample.timestamp = timestamp;
        chSysUnlock();
        chEvtBroadcastFlags(&sensor_events, SENSOR_EVENT_MS5611);
    }
}

static THD_WORKING_AREA(i2c_sensors_wa, 256);
static THD_FUNCTION(i2c_sensors, arg)
{
    chRegSetThreadName("onboard-sensors-i2c");
    (void)arg;
    static const I2CConfig i2c_cfg = {
        .op_mode = OPMODE_I2C,
        .clock_speed = 400000,
        .duty_cycle = FAST_DUTY_CYCLE_2
    };
    I2CDriver *i2c_driver = &I2CD1;

    i2cStart(i2c_driver, &i2c_cfg);

    // Barometer setup

    static ms5611_t barometer;

    i2cAcquireBus(i2c_driver);
    int init = ms5611_i2c_init(&barometer, i2c_driver, 0);
    i2cReleaseBus(i2c_driver);
    if (init != 0) {
        log_error("ms5611 init failed");
    }

    chThdCreateStatic(i2c_barometer_wa, sizeof(i2c_barometer_wa), THD_PRIO_SENSOR_DRV_I2C_BARO, i2c_barometer, &barometer);

    // High-g accelerometer setup

    static h3lis331dl_t high_g_acc;
    h3lis331dl_init_using_i2c(&high_g_acc, i2c_driver, H3LIS331DL_ADDR_SA0_HIGH);
    i2cAcquireBus(i2c_driver);
    if (!h3lis331dl_ping(&high_g_acc)) {
        log_error("h3lis331dl init failed");
    }
    i2cReleaseBus(i2c_driver);

    onboard_h3lis331dl_acc_sample.sensor = NULL; // todo
    i2cAcquireBus(i2c_driver);
    h3lis331dl_setup(&high_g_acc, H3LIS331DL_CONFIG_ODR_400HZ | H3LIS331DL_CONFIG_FS_400G);
    i2cReleaseBus(i2c_driver);

    // Magnetometer setup

    onboard_hmc5883l_mag_sample.sensor = NULL; // todo
    static hmc5883l_t magnetometer;
    hmc5883l_init(&magnetometer, i2c_driver);
    i2cAcquireBus(i2c_driver);
    if (!hmc5883l_ping(&magnetometer)) {
        log_error("hmc5883l init failed");
    }
    i2cReleaseBus(i2c_driver);

    i2cAcquireBus(i2c_driver);
    hmc5883l_setup(&magnetometer, HMC5883L_SAMPLE_AVG_8 | HMC5883L_GAIN_230 | HMC5883L_RATE_HZ_75);
    i2cReleaseBus(i2c_driver);

    // Event setup

    static event_listener_t sensor_int;
    chEvtRegisterMaskWithFlags(&exti_events, &sensor_int,
                               (eventmask_t)EXTI_INTERRUPT_EVENT,
                               (eventflags_t)EXTI_EVENT_H3LIS331DL_INT | EXTI_EVENT_HMC5883L_DRDY);

    while (1) {
        chEvtWaitAny(EXTI_INTERRUPT_EVENT);
        timestamp_t t = timestamp_get();
        eventflags_t event_flag = chEvtGetAndClearFlags(&sensor_int);
        if (event_flag & EXTI_EVENT_H3LIS331DL_INT) {
            static float acc[3];
            i2cAcquireBus(i2c_driver);
            h3lis331dl_read(&high_g_acc, acc);
            i2cReleaseBus(i2c_driver);
            chSysLock();
            onboard_h3lis331dl_acc_sample.acceleration[0] = acc[0];
            onboard_h3lis331dl_acc_sample.acceleration[1] = acc[1];
            onboard_h3lis331dl_acc_sample.acceleration[2] = acc[2];
            onboard_h3lis331dl_acc_sample.timestamp = t;
            chSysUnlock();
            chEvtBroadcastFlags(&sensor_events, SENSOR_EVENT_H3LIS331DL);
        }
        if (event_flag & EXTI_EVENT_HMC5883L_DRDY) {
            static float mag[3];
            i2cAcquireBus(i2c_driver);
            hmc5883l_read(&magnetometer, mag);
            i2cReleaseBus(i2c_driver);
            chSysLock();
            onboard_hmc5883l_mag_sample.magnetic_field[0] = mag[0];
            onboard_hmc5883l_mag_sample.magnetic_field[1] = mag[1];
            onboard_hmc5883l_mag_sample.magnetic_field[2] = mag[2];
            onboard_hmc5883l_mag_sample.timestamp = t;
            chSysUnlock();
            chEvtBroadcastFlags(&sensor_events, SENSOR_EVENT_HMC5883L);
        }
    }
}


void onboard_sensors_start(void)
{
    chEvtObjectInit(&sensor_events);
    exti_setup();
    chThdCreateStatic(spi_sensors_wa, sizeof(spi_sensors_wa), THD_PRIO_SENSOR_DRV_SPI, spi_sensors, NULL);
    chThdCreateStatic(i2c_sensors_wa, sizeof(i2c_sensors_wa), THD_PRIO_SENSOR_DRV_I2C, i2c_sensors, NULL);
}
