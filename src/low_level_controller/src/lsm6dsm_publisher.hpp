#ifndef LSM6DSM_PUBLISHER_HPP
#define LSM6DSM_PUBLISHER_HPP

#include <msgbus/msgbus.hpp>
#include <sensors.hpp>
#include <hal.h>
#include <Eigen/Dense>

void lsm6dsm_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board);

#endif /* LSM6DSM_PUBLISHER_HPP */
