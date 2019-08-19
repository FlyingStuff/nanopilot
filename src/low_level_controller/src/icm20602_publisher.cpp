#include "icm20602_publisher.hpp"
#include <drivers/icm20602.h>
#include "log.h"
#include "thread_prio.h"
#include <Eigen/Geometry>

static icm20602_t _icm20602_dev;
static Eigen::Matrix3f _R_sensor_to_board;


static THD_WORKING_AREA(icm20602_publisher_wa, 1800);
static THD_FUNCTION(icm20602_publisher, arg)
{
    (void)arg;
    chRegSetThreadName("icm20602_publisher");
    float temperature;
    Eigen::Vector3f rate_gyro_sensor, acc_sensor;
    Eigen::Quaternionf accumulated_angle(1, 0, 0, 0);
    if (!icm20602_ping(&_icm20602_dev)) {
        log_error("icm20602 ping failed");
        return;
    }
    icm20602_setup(&_icm20602_dev);
    timestamp_t prev_time = timestamp_get();
    while (true) {
        bool ok = icm20602_read(&_icm20602_dev, rate_gyro_sensor.data(),  acc_sensor.data(), &temperature);
        if (ok){
            rate_gyro_sample_t rate_gyro_sample;
            timestamp_t now = timestamp_get();
            float dt = timestamp_duration(prev_time, now);
            rate_gyro_sample.timestamp = now;
            Eigen::Map<Eigen::Vector3f> rate_gyro_board(rate_gyro_sample.rate);
            rate_gyro_board = _R_sensor_to_board * rate_gyro_sensor;
            accumulated_angle = accumulated_angle * Eigen::Quaternionf(1, dt*rate_gyro_board[0]/2, dt*rate_gyro_board[1]/2, dt*rate_gyro_board[2]/2);
            accumulated_angle.normalize();
            rate_gyro_sample.accumulated_angle.w = accumulated_angle.w();
            rate_gyro_sample.accumulated_angle.x = accumulated_angle.x();
            rate_gyro_sample.accumulated_angle.y = accumulated_angle.y();
            rate_gyro_sample.accumulated_angle.z = accumulated_angle.z();
            // log_debug("angle %f %f %f %f",
            //     rate_gyro_sample.accumulated_angle.w,
            //     rate_gyro_sample.accumulated_angle.x,
            //     rate_gyro_sample.accumulated_angle.y,
            //     rate_gyro_sample.accumulated_angle.z
            //     );
            rate_gyro.publish(rate_gyro_sample);

            prev_time = now;

            accelerometer_sample_t acc_sample;
            acc_sample.timestamp = timestamp_get();
            Eigen::Map<Eigen::Vector3f> acc_board(acc_sample.acceleration);
            acc_board = _R_sensor_to_board * acc_sensor;
            accelerometer.publish(acc_sample);
        }

        chThdSleepMilliseconds(1);
    }
}

void icm20602_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board)
{
    _R_sensor_to_board = R_sensor_to_board;
    icm20602_init(&_icm20602_dev, spi, config);
    chThdCreateStatic(icm20602_publisher_wa, sizeof(icm20602_publisher_wa), THD_PRIO_SPI_DRIVERS, icm20602_publisher, NULL);
}


