#include "lsm6dsm_publisher.hpp"
#include <drivers/lsm6dsm.h>
#include "log.h"
#include "thread_prio.h"
#include <Eigen/Geometry>

static lsm6dsm_t _lsm6dsm_dev;
static Eigen::Matrix3f _R_sensor_to_board;


static THD_WORKING_AREA(lsm6dsm_publisher_wa, 1800);
static THD_FUNCTION(lsm6dsm_publisher, arg)
{
    (void)arg;
    chRegSetThreadName("lsm6dsm_publisher");
    float temperature;
    Eigen::Vector3f rate_gyro_sensor, acc_sensor;
    Eigen::Quaternionf accumulated_angle(1, 0, 0, 0);

    if (!lsm6dsm_ping(&_lsm6dsm_dev)) {
        log_error("lsm6dsm ping failed");
        return;
    }
    lsm6dsm_setup(&_lsm6dsm_dev);
    timestamp_t prev_time = timestamp_get();
    imu_sample_t imu_sample;
    while (true) {

        auto update_status = lsm6dsm_read(&_lsm6dsm_dev, rate_gyro_sensor.data(),  acc_sensor.data(), &temperature);

        if (update_status & LSM6DSM_READ_GYRO_WAS_UPDATED){
            timestamp_t now = timestamp_get();
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
            if (update_status & LSM6DSM_READ_ACC_WAS_UPDATED){
                Eigen::Map<Eigen::Vector3f> acc_board(imu_sample.linear_acceleration);
                acc_board = _R_sensor_to_board * acc_sensor;
            }
            imu.publish(imu_sample);
            prev_time = now;
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


