
#include <stdint.h>
// #include <i2c.h>
#include "hal.h"

#define MS5611_OSR_256      0x00
#define MS5611_OSR_512      0x01
#define MS5611_OSR_1024     0x02
#define MS5611_OSR_2048     0x03
#define MS5611_OSR_4096     0x04

#define MS5611_I2C_ADDR1    0x76
#define MS5611_I2C_ADDR2    0x77

typedef struct {
    union {
        // i2c_dev_t i2c;
        // spi_dev_t spi;
        struct {
            I2CDriver *driver;
            uint8_t address;
        } i2c;
    } dev;
    enum {ms5611_i2c = 0, ms5611_spi = 1} mode;
    uint16_t prom[6];
} ms5611_t;

/** Initializes MS5611 device for I2C.
 *  The address depends on the value of the CSB pin.
 *  Returns 0 if initialization was successful. */
int ms5611_i2c_init(ms5611_t *ms5611, I2CDriver *driver, int csb_pin_value);

/** Initializes MS5611 device for SPI */
// void ms5611_spi_init(ms5611_t *ms5611, spi_dev_t *intf);

/** Resets the MS5611 device */
void ms5611_reset(ms5611_t *ms5611);

/** Read the PROM calibration values.
 *  Returns 1 if CRC check failed. */
int ms5611_prom_read(ms5611_t *ms5611);

/** Measure the pressure with the given osr over sampling rate.
 *  Returns 0 if measurement failed. */
uint32_t ms5611_press_adc_read(ms5611_t *ms5611, uint8_t osr);

/** Measure the temperature with the given osr over sampling rate.
 *  Returns 0 if measurement failed. */
uint32_t ms5611_temp_adc_read(ms5611_t *ms5611, uint8_t osr);

/** Calculates pressure from pressure and temperature adc values.
 *  Optional: Save temperature in 1/100 deg Celsius to p_temp pointer.
 *  Returns pressure in 1/100 mbar (= 1 Pa). */
uint32_t ms5611_calc_press(ms5611_t *ms5611, uint32_t raw_p, uint32_t raw_t, int32_t *p_temp);

/** Calculates temperature from adc value. */
int32_t ms5611_calc_temp(ms5611_t *ms5611, uint32_t raw_t);
