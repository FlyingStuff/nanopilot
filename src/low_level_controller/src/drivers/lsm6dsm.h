#ifndef LSM6DSM_H
#define LSM6DSM_H

#include <hal.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    SPIDriver *driver;
    SPIConfig driver_config;
    float gyro_FS;
    uint16_t acc_FS;
} lsm6dsm_t;


/* Initializes the LSM6DSM device datastructure */
void lsm6dsm_init(lsm6dsm_t *dev, SPIDriver *driver, const SPIConfig *config);

/* Returns true, if device ID is correct */
bool lsm6dsm_ping(lsm6dsm_t *dev);

/* LSM6DSM setup for given configuration options */
void lsm6dsm_setup(lsm6dsm_t *dev);

/* Reads gyro rates {x,y,z} in rad/s, accelerometer {x,y,z} in m/s
 * and temperature in deg Celsius. Returns the set of variables that were updated.
 */
#define LSM6DSM_READ_GYRO_WAS_UPDATED (1<<0)
#define LSM6DSM_READ_ACC_WAS_UPDATED (1<<1)
#define LSM6DSM_READ_TEMP_WAS_UPDATED (1<<2)
int lsm6dsm_read(lsm6dsm_t *dev, float *gyro, float *acc, float *temperature);


#ifdef __cplusplus
}
#endif

#endif /* LSM6DSM_H */