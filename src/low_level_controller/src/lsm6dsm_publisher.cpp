#include "lsm6dsm_publisher.hpp"
#include <drivers/lsm6dsm.h>
#include "log.h"
#include "thread_prio.h"

static lsm6dsm_t _lsm6dsm_dev;
static Eigen::Matrix3f _R_sensor_to_board;


static THD_WORKING_AREA(lsm6dsm_publisher_wa, 800);
static THD_FUNCTION(lsm6dsm_publisher, arg)
{
    (void)arg;
    chRegSetThreadName("lsm6dsm_publisher");
    float temperature;
    Eigen::Vector3f rate_gyro_sensor, acc_sensor;

    lsm6dsm_setup(&_lsm6dsm_dev);
    while (true) {

        auto update_status = lsm6dsm_read(&_lsm6dsm_dev, rate_gyro_sensor.data(),  acc_sensor.data(), &temperature);

        if (update_status & LSM6DSM_READ_GYRO_WAS_UPDATED){
            rate_gyro_sample_t rate_gyro_sample;
            rate_gyro_sample.timestamp = timestamp_get();
            Map<Vector3f> rate_gyro_board(&rate_gyro_sample.rate);
            rate_gyro_board = _R_sensor_to_board * rate_gyro_sensor;
            rate_gyro.publish(rate_gyro_sample);

        }


        if (update_status & LSM6DSM_READ_ACC_WAS_UPDATED){
            accelerometer_sample_t acc_sample;
            acc_sample.timestamp = timestamp_get();
            Map<Vector3f> acc_board(&acc_sample.rate);
            acc_board = _R_sensor_to_board * acc_sensor;
            accelerometer.publish(acc_sample);
        }
        chThdSleepMilliseconds(1);
    }
}

void lsm6dsm_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board)
{
    _R_sensor_to_board = R_sensor_to_board;
    lsm6dsm_init(&_lsm6dsm_dev, spi, config);
    chThdCreateStatic(lsm6dsm_publisher_wa, sizeof(lsm6dsm_publisher_wa), THD_PRIO_SPI_DRIVERS, lsm6dsm_publisher, NULL);
}


