#include <ch.h>
#include "mpu60X0.h"

#include "exti.h"

float gyro[3];
float acc[3];
float temp;

#define SENSOR_INTERRUPT_EVENT 1

static THD_WORKING_AREA(spi_sensors_wa, 128);
static THD_FUNCTION(spi_sensors, arg)
{
    (void)arg;
    chRegSetThreadName("onboard-sensors-spi");
    static event_listener_t sensor_int;
    chEvtRegisterMaskWithFlags(&exti_events, &sensor_int,
                               (eventmask_t)SENSOR_INTERRUPT_EVENT,
                               (eventflags_t)EXTI_EVENT_MPU6000_INT);
    /*
     * SPI1 configuration structure for MPU6000.
     * SPI1 is on APB2 @ 84MHz / 128 = 656.25kHz
     * CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
     */
    static SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOC,
        .sspad = GPIOC_MPU6000_CS,
        .cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
    };

    static mpu60X0_t mpu6000;
    mpu60X0_init_using_spi(&mpu6000, &SPID1);

    spiStart(&SPID1, &spi_cfg);
    if (!mpu60X0_ping(&mpu6000)) {
        error_set(ERROR_LEVEL_CRITICAL);
        return 0;
    }

    mpu60X0_setup(&mpu6000, MPU60X0_SAMPLE_RATE_DIV(0) | MPU60X0_LOW_PASS_FILTER_6);

    /* speed up SPI for sensor register reads (max 20MHz)
     * APB2 @ 84MHz / 8 = 10.5MHz
     */
    spi_cfg.cr1 = SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA;
    spiStart(&SPID1, &spi_cfg);
    // check that the sensor still pings
    if (!mpu60X0_ping(&mpu6000)) {
        error_set(ERROR_LEVEL_CRITICAL);
        return 0;
    }
    while (1) {
        // timestamp_t timestamp;
        palTogglePad(GPIOB, GPIOB_LED_STATUS);
        chEvtWaitAny(SENSOR_INTERRUPT_EVENT);
        mpu60X0_read(&mpu6000, gyro, acc, &temp);
    }
    return 0;
}

void onboard_sensors_start(void)
{
    exti_setup();
    chThdCreateStatic(spi_sensors_wa, sizeof(spi_sensors_wa), LOWPRIO, spi_sensors, NULL);
}
