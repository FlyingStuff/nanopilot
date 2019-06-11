#ifndef LIS3MDL_PUBLISHER_HPP
#define LIS3MDL_PUBLISHER_HPP

#include <msgbus/msgbus.hpp>
#include <sensors.hpp>
#include <hal.h>
#include <Eigen/Dense>

void lis3mdl_publisher_start(I2CDriver *i2c, uint8_t addr, const Eigen::Matrix3f &R_sensor_to_board);

#endif /* LIS3MDL_PUBLISHER_HPP */