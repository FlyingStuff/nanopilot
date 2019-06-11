#include "drivers/lis3mdl.h"
#include "log.h"
#include "thread_prio.h"
#include "lis3mdl_publisher.hpp"

static lis3mdl_t _lis3mdl_dev;
static Eigen::Matrix3f _R_sensor_to_board;


static THD_WORKING_AREA(lis3mdl_publisher_wa, 1800);
static THD_FUNCTION(lis3mdl_publisher, arg)
{
    (void)arg;
    chRegSetThreadName("lis3mdl_publisher");

    i2cAcquireBus(_lis3mdl_dev.i2c_driver);
    bool ping = lis3mdl_ping(&_lis3mdl_dev);
    i2cReleaseBus(_lis3mdl_dev.i2c_driver);
    if (!ping) {
        log_error("lis3mdl not responding to ping");
        return;
    }

    i2cAcquireBus(_lis3mdl_dev.i2c_driver);
    bool ok = lis3mdl_setup(&_lis3mdl_dev);
    i2cReleaseBus(_lis3mdl_dev.i2c_driver);
    if (!ok) {
        log_error("lis3mdl setup failed");
        return;
    }

    while (1) {
        while (1) {
            i2cAcquireBus(_lis3mdl_dev.i2c_driver);
            bool ready = lis3mdl_is_data_ready(&_lis3mdl_dev);
            i2cReleaseBus(_lis3mdl_dev.i2c_driver);
            if (ready)
                break;
            chThdSleepMilliseconds(1);
        }

        timestamp_t now = timestamp_get();
        Eigen::Vector3f mag_sensor;
        i2cAcquireBus(_lis3mdl_dev.i2c_driver);
        ok = lis3mdl_read(&_lis3mdl_dev, mag_sensor.data());
        i2cReleaseBus(_lis3mdl_dev.i2c_driver);
        if (ok) {
            magnetometer_sample_t mag_sample;
            mag_sample.timestamp = now;
            Eigen::Map<Eigen::Vector3f> mag_board(mag_sample.magnetic_field);
            mag_board = _R_sensor_to_board * mag_sensor;
            magnetometer.publish(mag_sample);
            // log_debug("lis3mdl ping %f %f %f", mag[0], mag[1], mag[2]);
        }

        // float temp;
        // i2cAcquireBus(_lis3mdl_dev.i2c_driver);
        // ok = lis3mdl_read_temperature(&_lis3mdl_dev, &temp);
        // i2cReleaseBus(_lis3mdl_dev.i2c_driver);
        // if (ok) {
        //     log_debug("lis3mdl temp %f", temp);
        // }
    }
}

void lis3mdl_publisher_start(I2CDriver *i2c, uint8_t addr, const Eigen::Matrix3f &R_sensor_to_board)
{
    _R_sensor_to_board = R_sensor_to_board;
    lis3mdl_init_using_i2c(&_lis3mdl_dev, i2c, addr);
    chThdCreateStatic(lis3mdl_publisher_wa, sizeof(lis3mdl_publisher_wa), THD_PRIO_I2C_DRIVERS, lis3mdl_publisher, NULL);
}
