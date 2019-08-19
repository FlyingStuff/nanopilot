/*
SPI: MSBit first, sampled on rising edge, max freq 10MHz


*/
#include "icm20602.h"
#include "math.h"

#define ICM20602_READ 0x80
#define ICM20602_WRITE 0x00

// Register addresses
#define WHO_AM_I 0x75
#define I2C_IF 0x70
#define I2C_IF_DIS 0x40
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_1_DEVICE_RESET 0x80
#define PWR_MGMT_1_CLKSEL_AUTO 0x01
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define GYRO_FS_SEL(x) (x<<3)
#define ACCEL_CONFIG 0x1C
#define ACCEL_FS_SEL(x) (x<<3)
#define ACCEL_CONFIG2 0x1d
#define INT_PIN_CFG 0x37
#define INT_PIN_CFG_LATCH_INT_EN 0x20
#define INT_ENABLE 0x38
#define INT_ENABLE_DATA_RDY 0x01
#define INT_STATUS 0x3A


#define STANDARD_GRAVITY 9.80665f // m/s

static float deg_p_sec_to_rad_p_sec(float x){
    return x*M_PI/180;
}

void icm20602_init(icm20602_t *dev, SPIDriver *driver, const SPIConfig *config)
{
    dev->driver = driver;
    dev->driver_config = *config;
    dev->gyro_FS = 0;
    dev->acc_FS = 0;
}

bool icm20602_ping(icm20602_t *dev)
{
    spiAcquireBus(dev->driver);
    spiStart(dev->driver, &dev->driver_config);
    spiSelect(dev->driver);
    uint8_t id_reg_addr = WHO_AM_I + ICM20602_READ;
    uint8_t id_reg_val;
    spiSend(dev->driver, 1, &id_reg_addr);
    spiReceive(dev->driver, 1, &id_reg_val);
    spiUnselect(dev->driver);
    spiReleaseBus(dev->driver);
    return id_reg_val == 0x12;
}

void icm20602_setup(icm20602_t *dev)
{
    spiAcquireBus(dev->driver);
    spiStart(dev->driver, &dev->driver_config);

    // reset device
    spiSelect(dev->driver);
    const uint8_t reset[2] = {PWR_MGMT_1 + ICM20602_WRITE, PWR_MGMT_1_DEVICE_RESET};
    spiSend(dev->driver, 2, &reset);
    spiUnselect(dev->driver);

    // 2ms startup time for reg read write
    chThdSleepMilliseconds(2);

    // disable I2C interface
    spiSelect(dev->driver);
    const uint8_t i2c_dis[2] = {I2C_IF + ICM20602_WRITE, I2C_IF_DIS};
    spiSend(dev->driver, 2, &i2c_dis);
    spiUnselect(dev->driver);

    // select auto clock source
    spiSelect(dev->driver);
    const uint8_t pwr_mgmt[2] = {PWR_MGMT_1 + ICM20602_WRITE, PWR_MGMT_1_CLKSEL_AUTO};
    spiSend(dev->driver, 2, &pwr_mgmt);
    spiUnselect(dev->driver);

    // sample div=0
    spiSelect(dev->driver);
    const uint8_t sample_div[2] = {SMPLRT_DIV + ICM20602_WRITE, 0};
    spiSend(dev->driver, 2, &sample_div);
    spiUnselect(dev->driver);

    // config DLPF_CFG=0
    spiSelect(dev->driver);
    const uint8_t config[2] = {CONFIG + ICM20602_WRITE, 0};
    spiSend(dev->driver, 2, &config);
    spiUnselect(dev->driver);

    // GYRO_CONFIG FCHOICE_B=0, FS_SEL=3
    // gyro @8kHz, 150Hz BW, 2000deg/s full scale
    dev->gyro_FS = deg_p_sec_to_rad_p_sec(2000);
    spiSelect(dev->driver);
    const uint8_t gyro_config[2] = {GYRO_CONFIG + ICM20602_WRITE, GYRO_FS_SEL(3)};
    spiSend(dev->driver, 2, &gyro_config);
    spiUnselect(dev->driver);

    // ACCEL_CONFIG FS_SEL=3
    // acc 16g full scale
    dev->acc_FS = 16.0f * STANDARD_GRAVITY;
    spiSelect(dev->driver);
    const uint8_t acc_config[2] = {ACCEL_CONFIG + ICM20602_WRITE, ACCEL_FS_SEL(3)};
    spiSend(dev->driver, 2, &acc_config);
    spiUnselect(dev->driver);

    // ACCEL_CONFIG2 ACCEL_FCHOICE_B=0, A_DLPF_CFG=0
    // 1kHz update rate, 218Hz BW
    spiSelect(dev->driver);
    const uint8_t acc_config2[2] = {ACCEL_CONFIG2 + ICM20602_WRITE, 0};
    spiSend(dev->driver, 2, &acc_config2);
    spiUnselect(dev->driver);

    // INT_PIN_CFG LATCH_INT_EN
    // push-pull, latching, interrupt is cleared by reading INT_STATUS
    spiSelect(dev->driver);
    const uint8_t int_pin_cfg[2] = {INT_PIN_CFG + ICM20602_WRITE, INT_PIN_CFG_LATCH_INT_EN};
    spiSend(dev->driver, 2, &int_pin_cfg);
    spiUnselect(dev->driver);

    // INT_ENABLE
    spiSelect(dev->driver);
    const uint8_t int_en[2] = {INT_ENABLE + ICM20602_WRITE, INT_ENABLE_DATA_RDY};
    spiSend(dev->driver, 2, &int_en);
    spiUnselect(dev->driver);

    spiReleaseBus(dev->driver);
    return;
}

static int16_t read_word(const uint8_t *buf)
{
    return ((int16_t)buf[0]) << 8 | buf[1];
}

bool icm20602_read(icm20602_t *dev, float *gyro, float *acc, float *temperature)
{
    spiAcquireBus(dev->driver);
    spiStart(dev->driver, &dev->driver_config);
    spiSelect(dev->driver);
    uint8_t reg_addr = INT_STATUS + ICM20602_READ;
    uint8_t val[15]; // status, 3x accel H&L, temp H&L, 3x gyro H&L
    spiSend(dev->driver, 1, &reg_addr);
    spiReceive(dev->driver, sizeof(val), &val);
    spiUnselect(dev->driver);
    spiReleaseBus(dev->driver);

    acc[0] = read_word(&val[1])*dev->acc_FS/(1<<15);;
    acc[1] = read_word(&val[3])*dev->acc_FS/(1<<15);;
    acc[2] = read_word(&val[5])*dev->acc_FS/(1<<15);;
    *temperature = read_word(&val[7]) * 0.00305997552f; // 1/326.8
    gyro[0] = read_word(&val[9])*dev->gyro_FS/(1<<15);
    gyro[1] = read_word(&val[11])*dev->gyro_FS/(1<<15);
    gyro[2] = read_word(&val[13])*dev->gyro_FS/(1<<15);

    return true;
}

