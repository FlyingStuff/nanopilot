
#include <stdint.h>
#include <stdbool.h>
#include "ms5611.h"

#define PROM_SENS       0
#define PROM_OFF        1
#define PROM_TCS        2
#define PROM_TCO        3
#define PROM_TREF       4
#define PROM_TEMPSENS   5

#define MS5611_CMD_ADC_PRESS(osr)  (0x40 | ((osr)<<1))
#define MS5611_CMD_ADC_TEMP(osr)   (0x50 | ((osr)<<1))
#define MS5611_CMD_RESET            0x1E
#define MS5611_CMD_ADC_READ         0x00
#define MS5611_CMD_PROM_READ_BASE   0XA0

#define SQUARE(x) ((x)*(x))

/* global variables */
const uint16_t ms5611_osr_dly_us[] = {600, 1170, 2280, 4540, 9040};

static int ms5611_prom_read_word_i2c(ms5611_t *ms5611, uint8_t addr, uint16_t *data);
static int ms5611_prom_read_word(ms5611_t *ms5611, uint8_t addr, uint16_t *data);
static uint16_t ms5611_crc4_update(uint16_t crc, uint16_t data);
static uint32_t ms5611_adc_read_i2c(ms5611_t *ms5611, uint8_t cmd, uint8_t osr);

int ms5611_i2c_init(ms5611_t *ms5611, I2CDriver *driver, int csb_pin_value)
{
    int err;
    uint8_t addr;

    ms5611->mode = ms5611_i2c;

    /* LSbit of addr is complementary of CSB pin */
    if (csb_pin_value == 1) {
        addr = 0x76;
    } else {
        addr = 0x77;
    }

    ms5611->dev.i2c.driver = driver;
    ms5611->dev.i2c.address = addr;

    err = ms5611_reset(ms5611);

    chThdSleepMilliseconds(5);

    if (err) {
        return err; /* error */
    }

    err = ms5611_prom_read(ms5611);

    return err;
}

int ms5611_reset(ms5611_t *ms5611)
{
    int err = 0;
    uint8_t reset_cmd = MS5611_CMD_RESET;

    if (ms5611->mode == ms5611_i2c) {
        uint8_t addr = ms5611->dev.i2c.address;
        I2CDriver *driver = ms5611->dev.i2c.driver;

        msg_t msg;
        i2cAcquireBus(driver);
        msg = i2cMasterTransmit(driver, addr, &reset_cmd, 1, NULL, 0);
        i2cReleaseBus(driver);

        if (msg != MSG_OK) {
            err = 1;
        }
    }

    return err;
}

static int ms5611_prom_read_word_i2c(ms5611_t *ms5611, uint8_t addr, uint16_t *data)
{
    uint8_t buf[2];
    msg_t msg;

    uint8_t i2c_addr = ms5611->dev.i2c.address;
    I2CDriver *driver = ms5611->dev.i2c.driver;

    i2cAcquireBus(driver);

    msg = i2cMasterTransmit(driver, i2c_addr, &addr, 1, buf, 2);

    i2cReleaseBus(driver);

    if (msg != MSG_OK) {
        return 1;
    }

    *data = (uint16_t) buf[1] | (buf[0] << 8);

    return 0;
}

static int ms5611_prom_read_word(ms5611_t *ms5611, uint8_t addr, uint16_t *data)
{
    if (ms5611->mode == ms5611_i2c) {
        return ms5611_prom_read_word_i2c(ms5611, addr, data);
    } else {
        // return ms5611_prom_read_word_spi(ms5611, addr, data);
        return 1;
    }
}

/* MS5611 4bit CRC calculation as described in AN520. */
static uint16_t ms5611_crc4_update(uint16_t crc, uint16_t data)
{
    uint8_t n_bit;

    crc ^= data>>8;

    for (n_bit = 8; n_bit > 0; n_bit--) {
        if (crc & (0x8000)) {
            crc = (crc << 1) ^ 0x3000;
        } else {
            crc = (crc << 1);
        }
    }

    crc ^= (data & 0x00ff);

    for (n_bit = 8; n_bit > 0; n_bit--) {
        if (crc & (0x8000)) {
            crc = (crc << 1) ^ 0x3000;
        } else {
            crc = (crc << 1);
        }
    }

    return crc;
}

int ms5611_prom_read(ms5611_t *ms5611)
{
    uint8_t addr;
    uint16_t crc_read, crc;

    addr = MS5611_CMD_PROM_READ_BASE;

    /* read reserved 16 bit for CRC */
    uint16_t d;
    if (ms5611_prom_read_word(ms5611, addr, &d)) {
        return 1;   /* read failed */
    }
    crc = ms5611_crc4_update(0, d);
    addr += 2;

    /* read PROM memory */
    uint8_t i;
    for (i = 0; i < 6; i++) {
        if (ms5611_prom_read_word(ms5611, addr, &d)) {
            return 1;   /* read failed */
        }
        crc = ms5611_crc4_update(crc, d);
        ms5611->prom[i] = d;
        addr += 2;
    }

    /* read CRC word */
    if (ms5611_prom_read_word(ms5611, addr, &crc_read)) {
        return 1;   /* read failed */
    }
    /* mask out CRC byte for calcualtion */
    crc = ms5611_crc4_update(crc, crc_read & 0xff00);
    /* get 4-bit CRC */
    crc = (crc>>12) & 0x000f;

    /* check 4-bit CRC */
    if ((crc_read & 0x000f) != crc) {
        return 2;   /* CRC error */
    }

    return 0;
}

static uint32_t ms5611_adc_read_i2c(ms5611_t *ms5611, uint8_t cmd, uint8_t osr)
{
    uint8_t buf[3];
    uint8_t addr = ms5611->dev.i2c.address;
    I2CDriver *driver = ms5611->dev.i2c.driver;
    msg_t msg;

    i2cAcquireBus(driver);

    /* send measurement command */
    msg = i2cMasterTransmit(driver, addr, &cmd, 1, NULL, 0);

    if (msg != MSG_OK) {
        i2cReleaseBus(driver);
        return 0;
    }

    /* sleep for needed conversion time */
    chThdSleepMilliseconds((ms5611_osr_dly_us[osr] - 1) / 1000 + 1);

    cmd = MS5611_CMD_ADC_READ;

    /* send ADC read command and read result */
    msg = i2cMasterTransmit(driver, addr, &cmd, 1, buf, 3);

    i2cReleaseBus(driver);

    if (msg != MSG_OK) {
        return 0;
    }

    /* setup 24bit result, MSByte received first */
    return (uint32_t) buf[2] | (buf[1] << 8) | (buf[0] << 16);
}

uint32_t ms5611_press_adc_read(ms5611_t *ms5611, uint8_t osr)
{
    uint8_t cmd;

    cmd = MS5611_CMD_ADC_PRESS(osr);

    if (ms5611->mode == ms5611_i2c) {
        return ms5611_adc_read_i2c(ms5611, cmd, osr);
    } else {
        // return ms5611_adc_read_spi(ms5611, cmd);
        return 0;
    }

}

uint32_t ms5611_temp_adc_read(ms5611_t *ms5611, uint8_t osr)
{
    uint8_t cmd;

    cmd = MS5611_CMD_ADC_TEMP(osr);

    if (ms5611->mode == ms5611_i2c) {
        return ms5611_adc_read_i2c(ms5611, cmd, osr);
    } else {
        // return ms5611_adc_read_spi(ms5611, cmd);
        return 0;
    }
}

int32_t ms5611_calc_temp(ms5611_t *ms5611, uint32_t raw_t)
{
    int32_t dt, temp;

    dt = (int32_t) raw_t - (ms5611->prom[PROM_TREF]<<8);
    temp = (int32_t) 2000 + (dt * ms5611->prom[PROM_TEMPSENS] / (1<<23));
    /* low temperature correcture, (temp < 20.00 C) */
    if (temp < 2000) {
        temp = temp - SQUARE(dt) / (1<<31);
    }
    return temp;
}

uint32_t ms5611_calc_press(ms5611_t *ms5611, uint32_t raw_p, uint32_t raw_t, int32_t *p_temp)
{
    int32_t dt, temp;
    int64_t off, sens;

    dt = (int32_t) raw_t - ms5611->prom[PROM_TREF] * (1<<8);
    temp = (int32_t) 2000 + dt * ms5611->prom[PROM_TEMPSENS] / (1<<23);

    off = (int64_t) ms5611->prom[PROM_OFF] * (1<<16) + ms5611->prom[PROM_TCO] * dt / (1<<7);
    sens = (int64_t) ms5611->prom[PROM_SENS] * (1<<15) + ms5611->prom[PROM_TCS] * dt / (1<<8);

    /* low temperature correcture, (temp < 20.00 C) */
    if (temp < 2000) {
        uint32_t t2, off2, sens2;

        t2 = (uint32_t) (SQUARE(dt) / (1<<31));
        off2 = (uint32_t) (5 * SQUARE(temp - 2000)) / 2;
        sens2 = off2 / 2;

        /* very low temperature correcture (temp < -15.00 C) */
        if (temp < -1500) {
            off2 = off2 + 7 * SQUARE(temp + 1500);
            sens2 = sens2 + ((11 * SQUARE(temp + 1500)) / 2);
        }

        /* correction */
        temp = temp - t2;
        off = off - off2;
        sens = sens - sens2;
    }

    if (p_temp != NULL) {
        *p_temp = temp;
    }

    /* calculate pressure */
    uint32_t p = (uint32_t) ((raw_p * sens / (1<<21)) - off) / (1<<15);
    return p;
}
