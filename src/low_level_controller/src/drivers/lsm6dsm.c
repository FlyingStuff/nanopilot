/*
SPI: MSBit first, sampled on rising edge


PING:
WHO_AM_I == 0X6A;


INT1_CTRL = INT1_DRDY_G // interrupt when new gyro data is ready

// ACC: 3.33KHz ODR, 1.5kHz analog bandwidth
CTRL1_XL = ODR_XL3 + ODR_XL0 // 3.33kHz
    + FS_XL0 + FS_XL1 // +-8g

// GYRO: 3.33kHz ODR
CTRL2_G = ODR_G3 + ODR_G0 // 3.33kHz
    + FS_G1 + FS_G0 // 2000dps
// system
CTRL3_C = BDU // update output register only after both high and low are read
    + IF_INC // auto increment register address
CTRL4_C = I2C_disable // disable I2C interface
CTRL5_C = 0 // default
CTRL6_C = 0 // default
CTRL7_G = 0 // default
CTRL8_XL = 0 // default
CTRL9_XL = 0 // default
CTRL10_C = 0 // default


*/
#include "lsm6dsm.h"


#define LSM6DSM_READ 0x80
#define LSM6DSM_WRITE 0x00

// Register addresses
#define WHO_AM_I 0x0F


void lsm6dsm_init(lsm6dsm_t *dev, SPIDriver *driver)
{
    dev->driver = driver;
}

bool lsm6dsm_ping(lsm6dsm_t *dev)
{
    spiSelect(dev->driver);
    uint8_t id_reg_addr = WHO_AM_I + LSM6DSM_READ;
    uint8_t id_reg_val;
    spiSend(dev->driver, 1, &id_reg_addr);
    spiReceive(dev->driver, 1, &id_reg_val);
    spiUnselect(dev->driver);
    return id_reg_val == 0x6A;
}

/* LSM6DSM setup for given configuration options */
void lsm6dsm_setup(lsm6dsm_t *dev);

/* Reads gyro rates {x,y,z} in rad/s, accelerometer {x,y,z} in m/s
 * and temperature in deg Celsius. Returns the set of variables that were updated.
 */
#define LSM6DSM_READ_GYRO_WAS_UPDATED (1<<0)
#define LSM6DSM_READ_ACC_WAS_UPDATED (1<<1)
#define LSM6DSM_READ_TEMP_WAS_UPDATED (1<<2)
int lsm6dsm_read(lsm6dsm_t *dev, float *gyro, float *acc, float *temperature);