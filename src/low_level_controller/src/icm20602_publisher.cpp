#include "icm20602_publisher.hpp"
#include <drivers/icm20602.h>
#include "log.h"
#include "thread_prio.h"
#include <Eigen/Geometry>

static icm20602_t _icm20602_dev;
static Eigen::Matrix3f _R_sensor_to_board;
static ioline_t _int_line;

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
        timestamp_t now = timestamp_get();
        bool ok = icm20602_read(&_icm20602_dev, rate_gyro_sensor.data(),  acc_sensor.data(), &temperature);
        if (ok){
            imu_sample_t imu_sample;
            float dt = timestamp_duration(prev_time, now);
            imu_sample.timestamp = now;
            Eigen::Map<Eigen::Vector3f> rate_gyro_board(imu_sample.angular_rate);
            rate_gyro_board = _R_sensor_to_board * rate_gyro_sensor;
            accumulated_angle = accumulated_angle * Eigen::Quaternionf(1, dt*rate_gyro_board[0]/2, dt*rate_gyro_board[1]/2, dt*rate_gyro_board[2]/2);
            accumulated_angle.normalize();
            imu_sample.accumulated_angle.w = accumulated_angle.w();
            imu_sample.accumulated_angle.x = accumulated_angle.x();
            imu_sample.accumulated_angle.y = accumulated_angle.y();
            imu_sample.accumulated_angle.z = accumulated_angle.z();
            Eigen::Map<Eigen::Vector3f> acc_board(imu_sample.linear_acceleration);
            acc_board = _R_sensor_to_board * acc_sensor;

            imu.publish(imu_sample);

            prev_time = now;
        }

        palWaitLineTimeout(_int_line, TIME_INFINITE);
    }
}

void icm20602_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board, ioline_t int_line)
{
    _int_line = int_line;
    palEnableLineEvent(int_line, PAL_EVENT_MODE_RISING_EDGE);
    _R_sensor_to_board = R_sensor_to_board;
    icm20602_init(&_icm20602_dev, spi, config);
    chThdCreateStatic(icm20602_publisher_wa, sizeof(icm20602_publisher_wa), THD_PRIO_SPI_DRIVERS, icm20602_publisher, NULL);
}


