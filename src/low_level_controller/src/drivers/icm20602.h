#ifndef ICM20602_H
#define ICM20602_H

#include <hal.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    SPIDriver *driver;
    SPIConfig driver_config;
    float gyro_FS;
    float acc_FS;
} icm20602_t;


/* Initializes the ICM20602 device datastructure */
void icm20602_init(icm20602_t *dev, SPIDriver *driver, const SPIConfig *config);

/* Returns true, if device ID is correct */
bool icm20602_ping(icm20602_t *dev);

/* ICM20602 setup for given configuration options. */
void icm20602_setup(icm20602_t *dev);

/* Reads gyro rates {x,y,z} in rad/s, accelerometer {x,y,z} in m/s
 * and temperature in deg Celsius. Returns true on success.
 */
bool icm20602_read(icm20602_t *dev, float *gyro, float *acc, float *temperature);


#ifdef __cplusplus
}
#endif

#endif /* ICM20602_H */