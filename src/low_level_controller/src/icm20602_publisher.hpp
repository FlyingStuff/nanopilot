#ifndef ICM20602_PUBLISHER_HPP
#define ICM20602_PUBLISHER_HPP

#include <msgbus/msgbus.hpp>
#include <sensors.hpp>
#include <hal.h>
#include <Eigen/Dense>
#include <parameter/parameter.h>

void icm20602_parameter_declare(parameter_namespace_t *parent_ns);
void icm20602_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board, ioline_t int_line);

#endif /* ICM20602_PUBLISHER_HPP */
