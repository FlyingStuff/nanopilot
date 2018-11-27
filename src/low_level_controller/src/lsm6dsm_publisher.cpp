#include "lsm6dsm_publisher.hpp"
#include <drivers/lsm6dsm.h>
#include "log.h"
#include "thread_prio.h"

static SPIDriver *spi_driver;

static THD_WORKING_AREA(lsm6dsm_publisher_wa, 800);
static THD_FUNCTION(lsm6dsm_publisher, arg)
{
    (void)arg;
    chRegSetThreadName("lsm6dsm_publisher");
    lsm6dsm_t lsm6dsm_dev;
    lsm6dsm_init(&lsm6dsm_dev, spi_driver);

    while (true) {
        log_debug("%d", (int)lsm6dsm_ping(&lsm6dsm_dev));
        chThdSleepMilliseconds(1000);
    }
}

void lsm6dsm_publisher_start(SPIDriver *spi)
{
    spi_driver = spi;
    chThdCreateStatic(lsm6dsm_publisher_wa, sizeof(lsm6dsm_publisher_wa), THD_PRIO_SPI_DRIVERS, lsm6dsm_publisher, NULL);
}


