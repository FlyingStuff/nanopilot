#ifndef LSM6DSM_PUBLISHER_HPP
#define LSM6DSM_PUBLISHER_HPP

#include <msgbus/msgbus.hpp>
#include <sensors.h>
#include <hal.h>

extern msgbus::Topic<rate_gyro_sample_t> rate_gyro;

void lsm6dsm_publisher_start(SPIDriver *spi, const SPIConfig *config);

#endif /* LSM6DSM_PUBLISHER_HPP */