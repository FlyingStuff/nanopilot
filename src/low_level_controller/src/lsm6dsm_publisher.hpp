#ifndef LSM6DSM_PUBLISHER_HPP
#define LSM6DSM_PUBLISHER_HPP

#include <msgbus/msgbus.hpp>
#include <sensors.h>
#include <hal.h>
#include <Eigen/Dense>

extern msgbus::Topic<rate_gyro_sample_t> rate_gyro;

void lsm6dsm_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board);

#endif /* LSM6DSM_PUBLISHER_HPP */
