#ifndef LIS3MDL_H
#define LIS3MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

typedef struct {
    I2CDriver *i2c_driver;
    uint8_t addr;
} lis3mdl_t;

void lis3mdl_init_using_i2c(lis3mdl_t *dev, I2CDriver *i2c_driver, uint8_t addr);
bool lis3mdl_ping(lis3mdl_t *dev);
bool lis3mdl_setup(lis3mdl_t *dev);
bool lis3mdl_is_data_ready(lis3mdl_t *dev);
bool lis3mdl_read(lis3mdl_t *dev, float mag_field[3]);
bool lis3mdl_read_temperature(lis3mdl_t *dev, float *temperature);

#ifdef __cplusplus
}
#endif

#endif /* LIS3MDL_H */