#include "lis3mdl.h"

// registers
#define REG_ADDR_AUTO_INCREMENT 0x80
#define REG_ADDR_WHO_AM_I 0x0F
#define REG_ADDR_CTRL_REG1 0x20
#define REG_ADDR_CTRL_REG2 0x21
#define REG_ADDR_CTRL_REG3 0x22
#define REG_ADDR_CTRL_REG4 0x23
#define REG_ADDR_CTRL_REG5 0x24
#define REG_ADDR_STATUS_REG 0x27
#define REG_ADDR_OUT_X_L 0x28
#define REG_ADDR_TEMP_OUT_L 0x2E

// bits
#define CTRL_REG1_TEMP_EN (1<<7)
#define CTRL_REG1_OM_ULTRA_HIGH_PERF (3<<5)
#define CTRL_REG1_FAST_ODR (1<<1)
#define CTRL_REG2_FULL_SCALE_4GAUSS 0
#define CTRL_REG4_OMZ_UTRA_HIGH_PERF (3<<2)
#define CTRL_REG5_BDU (1<<6)
#define STATUS_REG_ZYXDA (1<<3)


#define FULL_SCALE_4GAUSS_RES 1.4615609470914936e-08f // [Tesla/LSB] (1/6842/10**4)
#define TEMPERATURE_RES 0.125f // °C/LSB

void lis3mdl_init_using_i2c(lis3mdl_t *dev, I2CDriver *i2c_driver, uint8_t addr)
{
    dev->i2c_driver = i2c_driver;
    dev->addr = addr;
}

bool lis3mdl_ping(lis3mdl_t *dev)
{

    uint8_t reg_addr = REG_ADDR_WHO_AM_I;
    uint8_t buf;
    msg_t msg = i2cMasterTransmit(dev->i2c_driver, dev->addr, &reg_addr, 1, &buf, 1);
    if (msg == MSG_OK && buf == 0x3D) {
        return true;
    }
    return false;
}

bool lis3mdl_setup(lis3mdl_t *dev)
{
    uint8_t setup[6] = {
        REG_ADDR_CTRL_REG1 + REG_ADDR_AUTO_INCREMENT,
        CTRL_REG1_TEMP_EN + CTRL_REG1_OM_ULTRA_HIGH_PERF + CTRL_REG1_FAST_ODR, // CTRL1: data rate 155Hz
        CTRL_REG2_FULL_SCALE_4GAUSS, // CTRL2
        0, // CTRL3: continuous conversion
        CTRL_REG4_OMZ_UTRA_HIGH_PERF, // CTRL4, data LSByte first
        CTRL_REG5_BDU, // CTRL5, Block data update
    };
    msg_t msg = i2cMasterTransmit(dev->i2c_driver, dev->addr, setup, 6, NULL, 0);
    if (msg == MSG_OK) {
        return true;
    } else {
        return false;
    }
}

bool lis3mdl_is_data_ready(lis3mdl_t *dev)
{
    uint8_t reg = REG_ADDR_STATUS_REG;
    uint8_t stat;
    msg_t msg = i2cMasterTransmit(dev->i2c_driver, dev->addr, &reg, 1, &stat, 1);
    if (msg == MSG_OK && (stat & STATUS_REG_ZYXDA)) {
        return true;
    } else {
        return false;
    }
}

bool lis3mdl_read(lis3mdl_t *dev, float mag_field[3])
{
    uint8_t reg = REG_ADDR_OUT_X_L;
    uint8_t data[6];
    msg_t msg = i2cMasterTransmit(dev->i2c_driver, dev->addr, &reg, 1, data, 6);
    if (msg != MSG_OK) {
        return false;
    }
    mag_field[0] = FULL_SCALE_4GAUSS_RES*(int16_t)(data[0] + (data[1]<<8));
    mag_field[1] = FULL_SCALE_4GAUSS_RES*(int16_t)(data[2] + (data[3]<<8));
    mag_field[2] = FULL_SCALE_4GAUSS_RES*(int16_t)(data[4] + (data[5]<<8));
    return true;
}

bool lis3mdl_read_temperature(lis3mdl_t *dev, float *temperature)
{
    uint8_t reg = REG_ADDR_TEMP_OUT_L;
    uint8_t data[2];
    msg_t msg = i2cMasterTransmit(dev->i2c_driver, dev->addr, &reg, 1, data, 2);
    if (msg != MSG_OK) {
        return false;
    }
    *temperature = TEMPERATURE_RES*(int16_t)(data[0] + (data[1]<<8));
    return true;
}
