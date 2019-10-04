#ifndef ICM20602_PUBLISHER_HPP
#define ICM20602_PUBLISHER_HPP

#include <msgbus/msgbus.hpp>
#include <sensors.hpp>
#include <hal.h>
#include <Eigen/Dense>

void icm20602_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board, ioline_t int_line);

#endif /* ICM20602_PUBLISHER_HPP */
