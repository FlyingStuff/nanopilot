
#include "hal.h"

#if HAL_USE_PAL
/**
 * @brief     PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *                    This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config =
{
    {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
     VAL_GPIOA_ODR, VAL_GPIOA_AFRL, VAL_GPIOA_AFRH},
    {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
     VAL_GPIOB_ODR, VAL_GPIOB_AFRL, VAL_GPIOB_AFRH},
    {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
     VAL_GPIOC_ODR, VAL_GPIOC_AFRL, VAL_GPIOC_AFRH},
    {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
     VAL_GPIOD_ODR, VAL_GPIOD_AFRL, VAL_GPIOD_AFRH},
    {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
     VAL_GPIOE_ODR, VAL_GPIOE_AFRL, VAL_GPIOE_AFRH},
    {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR,
     VAL_GPIOF_ODR, VAL_GPIOF_AFRL, VAL_GPIOF_AFRH},
    {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR,
     VAL_GPIOG_ODR, VAL_GPIOG_AFRL, VAL_GPIOG_AFRH},
    {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
     VAL_GPIOH_ODR, VAL_GPIOH_AFRL, VAL_GPIOH_AFRH},
    {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
     VAL_GPIOI_ODR, VAL_GPIOI_AFRL, VAL_GPIOI_AFRH}
};
#endif


void __early_init(void) {
    stm32_clock_init();
}


#if HAL_USE_SDC
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {

    (void)sdcp;
    return !palReadPad(GPIOC, GPIOC_SDCARD_DETECT);
}

bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

    (void)sdcp;
    return FALSE;
}
#endif


void board_init(void) {
    board_sensor_pwr_en(true);
    board_io_pwr_en(true);
}


void board_io_pwr_en(bool en)
{
    chSysLock();
    if (en) {
        palSetPad(GPIOB, GPIOB_IO_POWER_EN);
        palSetPadMode(GPIOA, GPIOA_UART4_TX_CONN2, PAL_MODE_ALTERNATE(8) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOA, GPIOA_UART4_RX_CONN2, PAL_MODE_ALTERNATE(8) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOA, GPIOA_UART2_TX_CONN3, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOA, GPIOA_UART2_RX_CONN3, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOB, GPIOB_UART1_TX_CONN1, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOB, GPIOB_UART1_RX_CONN1, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOC, GPIOC_UART6_TX_CONN4, PAL_MODE_ALTERNATE(8) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOC, GPIOC_UART6_RX_CONN4, PAL_MODE_ALTERNATE(8) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOB, GPIOB_I2C2_SCL_CONN, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
        palSetPadMode(GPIOB, GPIOB_I2C2_SDA_CONN, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
    } else {
        palSetPadMode(GPIOA, GPIOA_UART4_TX_CONN2, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOA, GPIOA_UART4_RX_CONN2, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOA, GPIOA_UART2_TX_CONN3, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOA, GPIOA_UART2_RX_CONN3, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOB, GPIOB_UART1_TX_CONN1, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOB, GPIOB_UART1_RX_CONN1, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_UART6_TX_CONN4, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_UART6_RX_CONN4, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOB, GPIOB_I2C2_SCL_CONN, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOB, GPIOB_I2C2_SDA_CONN, PAL_MODE_INPUT_PULLDOWN);
        palClearPad(GPIOB, GPIOB_IO_POWER_EN);
    }
    chSysUnlock();
}


void board_sensor_pwr_en(bool en)
{
    chSysLock();
    if (en) {
        palSetPad(GPIOC, GPIOC_VCC_A_POWER_EN);
        palSetPadMode(GPIOA, GPIOA_SPI1_SCK, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOA, GPIOA_SPI1_MISO, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOA, GPIOA_SPI1_MOSI, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOC, GPIOC_MPU6000_FSYNC, PAL_STM32_MODE_OUTPUT | PAL_STM32_OSPEED_HIGHEST);
        palSetPad(GPIOC, GPIOC_MPU6000_CS);
        palSetPadMode(GPIOC, GPIOC_MPU6000_CS, PAL_STM32_MODE_OUTPUT | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(GPIOB, GPIOB_I2C1_SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
        palSetPadMode(GPIOB, GPIOB_I2C1_SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
    } else {
        palSetPadMode(GPIOA, GPIOA_SPI1_SCK, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOA, GPIOA_SPI1_MISO, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOA, GPIOA_SPI1_MOSI, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_MPU6000_FSYNC, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_MPU6000_CS, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOB, GPIOB_I2C1_SCL, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOB, GPIOB_I2C1_SDA, PAL_MODE_INPUT_PULLDOWN);
        palClearPad(GPIOC, GPIOC_VCC_A_POWER_EN);
    }
    chSysUnlock();
}


void board_sdcard_pwr_en(bool en)
{
    if (en) {
        palClearPad(GPIOC, GPIOC_SDCARD_POWER_EN);
        palSetPadMode(GPIOC, GPIOC_SDIO_D0, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOC, GPIOC_SDIO_D1, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOC, GPIOC_SDIO_D2, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOC, GPIOC_SDIO_D3, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST); // has external pullup
        palSetPadMode(GPIOC, GPIOC_SDIO_CK, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
        palSetPadMode(GPIOD, GPIOD_SDIO_CMD, PAL_MODE_ALTERNATE(12) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
    } else {
        palSetPadMode(GPIOC, GPIOC_SDIO_D0, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_SDIO_D1, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_SDIO_D2, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_SDIO_D3, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOC, GPIOC_SDIO_CK, PAL_MODE_INPUT_PULLDOWN);
        palSetPadMode(GPIOD, GPIOD_SDIO_CMD, PAL_MODE_INPUT_PULLDOWN);
        palSetPad(GPIOC, GPIOC_SDCARD_POWER_EN);
    }
}


void board_can_standby(bool en)
{
    (void)en;
// GPIOC_CAN_CONN_EN // high = standby
}


void panic_handler(const char *reason)
{
    (void)reason;
    palSetPad(GPIOA, GPIOA_LED_ERROR);
    palClearPad(GPIOB, GPIOB_LED_STATUS);
    palClearPad(GPIOA, GPIOA_LED_HEARTBEAT);
    palClearPad(GPIOB, GPIOB_LED_SDCARD);

    static volatile uint32_t ipsr;
    static volatile const char *msg;
    msg = reason;
    ipsr = __get_IPSR();

    (void)msg;
    (void)ipsr;
#ifdef DEBUG
    while (1);
#else
    NVIC_SystemReset();
#endif
}


void status_led_on(void)
{
    palSetPad(GPIOB, GPIOB_LED_STATUS);
}

void status_led_off(void)
{
    palClearPad(GPIOB, GPIOB_LED_STATUS);
}

void status_led_toggle(void)
{
    chSysLock();
    palTogglePad(GPIOB, GPIOB_LED_STATUS);
    chSysUnlock();
}


static int error_level_cnt[2] = {0,0};

void error_set(int level)
{
    if (level > ERROR_LEVEL_NORMAL && level <= ERROR_LEVEL_CRITICAL) {
        chSysLock();
        error_level_cnt[level - 1]++;
        chSysUnlock();
    }
}

void error_clear(int level)
{
    if (level > ERROR_LEVEL_NORMAL && level <= ERROR_LEVEL_CRITICAL) {
        chSysLock();
        if (error_level_cnt[level - 1] > 0) {
            error_level_cnt[level - 1]--;
        }
        chSysUnlock();
    }
}

int error_level_get(void)
{
    int lvl = ERROR_LEVEL_NORMAL;
    chSysLock();
    if (error_level_cnt[ERROR_LEVEL_CRITICAL - 1]) {
        lvl = ERROR_LEVEL_CRITICAL;
    } else if (error_level_cnt[ERROR_LEVEL_WARNING - 1]) {
        lvl = ERROR_LEVEL_WARNING;
    }
    chSysUnlock();
    return lvl;
}

bool safemode_active(void)
{
    return false;
}
