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
#include "math.h"

#define LSM6DSM_READ 0x80
#define LSM6DSM_WRITE 0x00

// Register addresses
#define WHO_AM_I 0x0F
#define INT1_CTRL 0x0D

#define CTRL1_XL 0x10
#define CTRL1_XL_ODR_XL3 (1<<7)
#define CTRL1_XL_ODR_XL0 (1<<4)
#define CTRL1_XL_FS_XL1 (1<<3)
#define CTRL1_XL_FS_XL0 (1<<2)

#define CTRL2_G 0x11
#define CTRL2_G_ODR_G3 (1<<7)
#define CTRL2_G_ODR_G0 (1<<4)
#define CTRL2_G_FS_G1 (1<<3)
#define CTRL2_G_FS_G0 (1<<2)

#define CTRL3_C 0x12
#define CTRL3_C_BDU (1<<6)
#define CTRL3_C_IF_INC (1<<2)

#define CTRL4_C 0x13
#define CTRL4_C_I2C_DISABLE (1<<2)

#define LSM6DSM_READ_GYRO_WAS_UPDATED (1<<0)
#define LSM6DSM_READ_ACC_WAS_UPDATED (1<<1)
#define LSM6DSM_READ_TEMP_WAS_UPDATED (1<<2)

#define LSM6DSM_STATUS_REG 0x1E
#define STANDARD_GRAVITY 9.80665f // m/s



/* Converts two bytes in a word */
static int16_t read_word(const uint8_t *buf)
{
    return ((int16_t)((int16_t)buf[1]) << 8 | buf[0]);
}

float deg_p_sec_to_rad_p_sec(float x){
    return x*M_PI/180;
}


void lsm6dsm_init(lsm6dsm_t *dev, SPIDriver *driver, const SPIConfig *config)
{
    dev->driver = driver;
    dev->driver_config = *config;
    // GYRO_FS = 2000;
    dev->gyro_FS = deg_p_sec_to_rad_p_sec(2000);
    dev->acc_FS = 8;
}


/* LSM6DSM ping. Returns true if device found. */
bool lsm6dsm_ping(lsm6dsm_t *dev)
{
    spiAcquireBus(dev->driver);
    spiStart(dev->driver, &dev->driver_config);
    spiSelect(dev->driver);
    uint8_t id_reg_addr = WHO_AM_I + LSM6DSM_READ;
    uint8_t id_reg_val;
    spiSend(dev->driver, 1, &id_reg_addr);
    spiReceive(dev->driver, 1, &id_reg_val);
    spiUnselect(dev->driver);
    spiReleaseBus(dev->driver);
    return id_reg_val == 0x6A;
}


/* LSM6DSM setup for given configuration options */
void lsm6dsm_setup(lsm6dsm_t *dev)
{
    spiAcquireBus(dev->driver);
    spiStart(dev->driver, &dev->driver_config);
    spiSelect(dev->driver);
    uint8_t buf[5] = {CTRL1_XL + LSM6DSM_WRITE,
                    CTRL1_XL_ODR_XL3 | CTRL1_XL_ODR_XL0 | CTRL1_XL_FS_XL0 | CTRL1_XL_FS_XL1,
                    CTRL2_G_ODR_G3 | CTRL2_G_ODR_G0 | CTRL2_G_FS_G1 | CTRL2_G_FS_G0,
                    CTRL3_C_BDU | CTRL3_C_IF_INC,
                    CTRL4_C_I2C_DISABLE};
    spiSend(dev->driver, 5, buf);
    spiUnselect(dev->driver);
    spiReleaseBus(dev->driver);
}
#include "log.h"



/* Reads gyro rates {x,y,z} in rad/s, accelerometer {x,y,z} in m/s
 * and temperature in deg Celsius. Returns the set of variables that were updated.
 */
int lsm6dsm_read(lsm6dsm_t *dev, float *gyro, float *acc, float *temperature){
    const uint8_t size_buf = 1 + 1 + 2 + 3 * 2 + 3 * 2;
    uint8_t buf[size_buf];
    uint8_t reg = LSM6DSM_STATUS_REG + LSM6DSM_READ;

    spiAcquireBus(dev->driver);
    spiStart(dev->driver, &dev->driver_config);
    spiSelect(dev->driver);

    spiSend(dev->driver, 1, &reg);
    spiReceive(dev->driver, size_buf, buf);

    spiUnselect(dev->driver);
    spiReleaseBus(dev->driver);

    *temperature = (float)read_word(&buf[2])/256 + 25;

    gyro[0] = (float)(dev->gyro_FS * read_word(&buf[2+2]))/(1<<15);
    gyro[1] = (float)(dev->gyro_FS * read_word(&buf[2+4]))/(1<<15);
    gyro[2] = (float)(dev->gyro_FS * read_word(&buf[2+6]))/(1<<15);

    acc[0] = (float) dev->acc_FS * read_word(&buf[2+8]) * STANDARD_GRAVITY/(1<<15);
    acc[1] = (float) dev->acc_FS * read_word(&buf[2+10]) * STANDARD_GRAVITY/(1<<15);
    acc[2] = (float) dev->acc_FS * read_word(&buf[2+12]) * STANDARD_GRAVITY/(1<<15);




    return 0;
}