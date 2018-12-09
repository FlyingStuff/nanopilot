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
    float gyro[3], acc[3], temperature;
    lsm6dsm_setup(&_lsm6dsm_dev);
    while (true) {
        lsm6dsm_read(&_lsm6dsm_dev, gyro, acc, &temperature);

        chThdSleepMilliseconds(1);
    }
}

void lsm6dsm_publisher_start(SPIDriver *spi, const SPIConfig *config, const Eigen::Matrix3f &R_sensor_to_board)
{
    _R_sensor_to_board = R_sensor_to_board;
    lsm6dsm_init(&_lsm6dsm_dev, spi, config);
    chThdCreateStatic(lsm6dsm_publisher_wa, sizeof(lsm6dsm_publisher_wa), THD_PRIO_SPI_DRIVERS, lsm6dsm_publisher, NULL);
}


