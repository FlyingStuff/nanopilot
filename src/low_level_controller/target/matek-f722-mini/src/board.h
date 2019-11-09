#ifndef BOARD_H
#define BOARD_H

// https://github.com/betaflight/betaflight/tree/master/src/main/target/MATEKF722SE

/*
 * Board identifier.
 */
#define BOARD_NAME                  "Matek-F722-mini"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#define STM32_LSECLK                0U

#define STM32_LSEDRV                (3U << 3U)

#define STM32_HSECLK                8000000U

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U


/*
 * MCU type as defined in the ST header.
 */
#define STM32F722xx

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0_UART2_CTS        0U // TX4
#define GPIOA_PIN1_UART2_RTS        1U // RX4
#define GPIOA_PIN2_UART2_TX         2U // TX2
#define GPIOA_PIN3_UART2_RX         3U // RX2
#define GPIOA_PIN4_ADC_EXT          4U
#define GPIOA_PIN5_SPI_GYRO_SCK     5U
#define GPIOA_PIN6_SPI_GYRO_MISO    6U
#define GPIOA_PIN7_SPI_GYRO_MOSI    7U
#define GPIOA_PIN8_LED_STRIP_PWM    8U // TIM1_CH1
#define GPIOA_PIN9_TX1              9U
#define GPIOA_PIN10_RX1             10U
#define GPIOA_PIN11_USB_DM          11U
#define GPIOA_PIN12_USB_DP          12U
#define GPIOA_SWDIO_LED2            13U
#define GPIOA_SWCLK_LED1            14U
#define GPIOA_PIN15_MOTOR5          15U // TIM2 CH1

#define GPIOB_PIN0_MOTOR3           0U // TIM3 CH3
#define GPIOB_PIN1_MOTOR4           1U // TIM3 CH4
#define GPIOB_PIN2_GYRO1_CS         2U
#define GPIOB_PIN3_MOTOR6           3U // TIM2 CH2
#define GPIOB_PIN4_MOTOR1           4U // TIM3 CH1
#define GPIOB_PIN5_MOTOR2           5U // TIM3 CH2
#define GPIOB_PIN6_MOTOR7           6U // TIM4 CH1
#define GPIOB_PIN7_MOTOR8           7U // TIM4 CH2
#define GPIOB_PIN8_I2C_SCL          8U
#define GPIOB_PIM9_I2C_SDA          9U
#define GPIOB_PIN10_TX3_MOTOR9      10U // TIM2 CH3
#define GPIOB_PIN11_RX3_MOTOR10     11U // TIM2 CH4
#define GPIOB_PIN12_OSD_CS          12U
#define GPIOB_PIN13_SPI_OSD_SCLK    13U
#define GPIOB_PIN14_SPI_OSD_MISO    14U
#define GPIOB_PIN15_SPI_OSD_MOSI    15U

#define GPIOC_PIN0_ADC_RSSI         0U
#define GPIOC_PIN1_ADC_CURRENT      1U
#define GPIOC_PIN2_ADC_VOLTAGE      2U
#define GPIOC_PIN3_GYRO2_INT        3U
#define GPIOC_PIN4_GYRO1_INT        4U
#define GPIOC_PIN5_NC               5U
#define GPIOC_PIN6_TX6              6U
#define GPIOC_PIN7_RX6              7U
#define GPIOC_PIN8_VBAT_SW          8U
#define GPIOC_PIN9_CAM_SEL          9U
#define GPIOC_PIN10_SPI_FLASH_SCK   10U
#define GPIOC_PIN11_SPI_FLASH_MISO  11U
#define GPIOC_PIN12_SPI_FLASH_MOSI  12U
#define GPIOC_PIN13_BEEPER          13U
#define GPIOC_PIN14_USB_DET         14U
#define GPIOC_PIN15_GYRO2_CS        15U

#define GPIOD_PIN0_NC               0U
#define GPIOD_PIN1_NC               1U
#define GPIOD_PIN2_FLASH_CS         2U
#define GPIOD_PIN3_NC               3U
#define GPIOD_PIN4_NC               4U
#define GPIOD_PIN5_NC               5U
#define GPIOD_PIN6_NC               6U
#define GPIOD_PIN7_NC               7U
#define GPIOD_PIN8_NC               8U
#define GPIOD_PIN9_NC               9U
#define GPIOD_PIN10_NC              10U
#define GPIOD_PIN11_NC              11U
#define GPIOD_PIN12_NC              12U
#define GPIOD_PIN13_NC              13U
#define GPIOD_PIN14_NC              14U
#define GPIOD_PIN15_NC              15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U



/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))


#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN2_UART2_TX) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN3_UART2_RX) |\
                                     PIN_MODE_ANALOG(GPIOA_PIN4_ADC_EXT) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN5_SPI_GYRO_SCK) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN6_SPI_GYRO_MISO) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN7_SPI_GYRO_MOSI) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN8_LED_STRIP_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN9_TX1) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN10_RX1) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN11_USB_DM) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN12_USB_DP) |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO_LED2) |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK_LED1) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN15_MOTOR5))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2_UART2_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3_UART2_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4_ADC_EXT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5_SPI_GYRO_SCK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6_SPI_GYRO_MISO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7_SPI_GYRO_MOSI) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8_LED_STRIP_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN9_TX1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10_RX1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN11_USB_DM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN12_USB_DP) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO_LED2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK_LED1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15_MOTOR5))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN2_UART2_TX) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN3_UART2_RX) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN4_ADC_EXT) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN5_SPI_GYRO_SCK) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN6_SPI_GYRO_MISO) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN7_SPI_GYRO_MOSI) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN8_LED_STRIP_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN9_TX1) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN10_RX1) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN11_USB_DM) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN12_USB_DP) |\
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO_LED2) |\
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK_LED1) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN15_MOTOR5))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN2_UART2_TX) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN3_UART2_RX) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4_ADC_EXT) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5_SPI_GYRO_SCK) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN6_SPI_GYRO_MISO) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN7_SPI_GYRO_MOSI) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8_LED_STRIP_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN9_TX1) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN10_RX1) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN11_USB_DM) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN12_USB_DP) |\
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO_LED2) |\
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK_LED1) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN15_MOTOR5))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_ODR_HIGH(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_ODR_HIGH(GPIOA_PIN2_UART2_TX) |\
                                     PIN_ODR_HIGH(GPIOA_PIN3_UART2_RX) |\
                                     PIN_ODR_HIGH(GPIOA_PIN4_ADC_EXT) |\
                                     PIN_ODR_HIGH(GPIOA_PIN5_SPI_GYRO_SCK) |\
                                     PIN_ODR_HIGH(GPIOA_PIN6_SPI_GYRO_MISO) |\
                                     PIN_ODR_HIGH(GPIOA_PIN7_SPI_GYRO_MOSI) |\
                                     PIN_ODR_HIGH(GPIOA_PIN8_LED_STRIP_PWM) |\
                                     PIN_ODR_HIGH(GPIOA_PIN9_TX1) |\
                                     PIN_ODR_HIGH(GPIOA_PIN10_RX1) |\
                                     PIN_ODR_HIGH(GPIOA_PIN11_USB_DM) |\
                                     PIN_ODR_HIGH(GPIOA_PIN12_USB_DP) |\
                                     PIN_ODR_HIGH(GPIOA_SWDIO_LED2) |\
                                     PIN_ODR_HIGH(GPIOA_SWCLK_LED1) |\
                                     PIN_ODR_HIGH(GPIOA_PIN15_MOTOR5))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0_UART2_CTS, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN1_UART2_RTS, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN2_UART2_TX, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN3_UART2_RX, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN4_ADC_EXT, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN5_SPI_GYRO_SCK, 5U) |\
                                     PIN_AFIO_AF(GPIOA_PIN6_SPI_GYRO_MISO, 5U) |\
                                     PIN_AFIO_AF(GPIOA_PIN7_SPI_GYRO_MOSI, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8_LED_STRIP_PWM, 1U) |\
                                     PIN_AFIO_AF(GPIOA_PIN9_TX1, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN10_RX1, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN11_USB_DM, 10U) |\
                                     PIN_AFIO_AF(GPIOA_PIN12_USB_DP, 10U) |\
                                     PIN_AFIO_AF(GPIOA_SWDIO_LED2, 0U) |\
                                     PIN_AFIO_AF(GPIOA_SWCLK_LED1, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN15_MOTOR5, 1U))





#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_PIN0_MOTOR3) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN1_MOTOR4) |\
                                     PIN_MODE_OUTPUT(GPIOB_PIN2_GYRO1_CS) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN3_MOTOR6) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN4_MOTOR1) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN5_MOTOR2) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN6_MOTOR7) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN7_MOTOR8) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN8_I2C_SCL) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIM9_I2C_SDA) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN10_TX3_MOTOR9) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN11_RX3_MOTOR10) |\
                                     PIN_MODE_OUTPUT(GPIOB_PIN12_OSD_CS) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN13_SPI_OSD_SCLK) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN14_SPI_OSD_MISO) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN15_SPI_OSD_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0_MOTOR3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1_MOTOR4) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2_GYRO1_CS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN3_MOTOR6) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4_MOTOR1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5_MOTOR2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6_MOTOR7) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7_MOTOR8) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN8_I2C_SCL) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIM9_I2C_SDA) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10_TX3_MOTOR9) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11_RX3_MOTOR10) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12_OSD_CS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13_SPI_OSD_SCLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14_SPI_OSD_MISO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15_SPI_OSD_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_PIN0_MOTOR3) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN1_MOTOR4) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN2_GYRO1_CS) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN3_MOTOR6) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN4_MOTOR1) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN5_MOTOR2) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN6_MOTOR7) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN7_MOTOR8) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN8_I2C_SCL) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIM9_I2C_SDA) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN10_TX3_MOTOR9) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN11_RX3_MOTOR10) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN12_OSD_CS) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN13_SPI_OSD_SCLK) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN14_SPI_OSD_MISO) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN15_SPI_OSD_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PIN0_MOTOR3) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1_MOTOR4) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN2_GYRO1_CS) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN3_MOTOR6) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4_MOTOR1) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5_MOTOR2) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6_MOTOR7) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN7_MOTOR8) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN8_I2C_SCL) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIM9_I2C_SDA) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10_TX3_MOTOR9) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN11_RX3_MOTOR10) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12_OSD_CS) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13_SPI_OSD_SCLK) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN14_SPI_OSD_MISO) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15_SPI_OSD_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0_MOTOR3) |\
                                     PIN_ODR_HIGH(GPIOB_PIN1_MOTOR4) |\
                                     PIN_ODR_HIGH(GPIOB_PIN2_GYRO1_CS) |\
                                     PIN_ODR_HIGH(GPIOB_PIN3_MOTOR6) |\
                                     PIN_ODR_HIGH(GPIOB_PIN4_MOTOR1) |\
                                     PIN_ODR_HIGH(GPIOB_PIN5_MOTOR2) |\
                                     PIN_ODR_HIGH(GPIOB_PIN6_MOTOR7) |\
                                     PIN_ODR_HIGH(GPIOB_PIN7_MOTOR8) |\
                                     PIN_ODR_HIGH(GPIOB_PIN8_I2C_SCL) |\
                                     PIN_ODR_HIGH(GPIOB_PIM9_I2C_SDA) |\
                                     PIN_ODR_HIGH(GPIOB_PIN10_TX3_MOTOR9) |\
                                     PIN_ODR_HIGH(GPIOB_PIN11_RX3_MOTOR10) |\
                                     PIN_ODR_HIGH(GPIOB_PIN12_OSD_CS) |\
                                     PIN_ODR_HIGH(GPIOB_PIN13_SPI_OSD_SCLK) |\
                                     PIN_ODR_HIGH(GPIOB_PIN14_SPI_OSD_MISO) |\
                                     PIN_ODR_HIGH(GPIOB_PIN15_SPI_OSD_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0_MOTOR3, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN1_MOTOR4, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN2_GYRO1_CS, 0U) |\
                                     PIN_AFIO_AF(GPIOB_PIN3_MOTOR6, 1U) |\
                                     PIN_AFIO_AF(GPIOB_PIN4_MOTOR1, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN5_MOTOR2, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN6_MOTOR7, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN7_MOTOR8, 2U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8_I2C_SCL, 4U) |\
                                     PIN_AFIO_AF(GPIOB_PIM9_I2C_SDA, 4U) |\
                                     PIN_AFIO_AF(GPIOB_PIN10_TX3_MOTOR9, 1U) |\
                                     PIN_AFIO_AF(GPIOB_PIN11_RX3_MOTOR10, 1U) |\
                                     PIN_AFIO_AF(GPIOB_PIN12_OSD_CS, 0U) |\
                                     PIN_AFIO_AF(GPIOB_PIN13_SPI_OSD_SCLK, 5U) |\
                                     PIN_AFIO_AF(GPIOB_PIN14_SPI_OSD_MISO, 5U) |\
                                     PIN_AFIO_AF(GPIOB_PIN15_SPI_OSD_MOSI, 5U))





#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_PIN0_ADC_RSSI) |\
                                     PIN_MODE_ANALOG(GPIOC_PIN1_ADC_CURRENT) |\
                                     PIN_MODE_ANALOG(GPIOC_PIN2_ADC_VOLTAGE) |\
                                     PIN_MODE_INPUT(GPIOC_PIN3_GYRO2_INT) |\
                                     PIN_MODE_INPUT(GPIOC_PIN4_GYRO1_INT) |\
                                     PIN_MODE_INPUT(GPIOC_PIN5_NC) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN6_TX6) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN7_RX6) |\
                                     PIN_MODE_OUTPUT(GPIOC_PIN8_VBAT_SW) |\
                                     PIN_MODE_OUTPUT(GPIOC_PIN9_CAM_SEL) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN10_SPI_FLASH_SCK) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN11_SPI_FLASH_MISO) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN12_SPI_FLASH_MOSI) |\
                                     PIN_MODE_OUTPUT(GPIOC_PIN13_BEEPER) |\
                                     PIN_MODE_INPUT(GPIOC_PIN14_USB_DET) |\
                                     PIN_MODE_OUTPUT(GPIOC_PIN15_GYRO2_CS))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0_ADC_RSSI) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1_ADC_CURRENT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2_ADC_VOLTAGE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3_GYRO2_INT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4_GYRO1_INT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6_TX6) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7_RX6) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8_VBAT_SW) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9_CAM_SEL) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10_SPI_FLASH_SCK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11_SPI_FLASH_MISO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12_SPI_FLASH_MOSI) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13_BEEPER) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14_USB_DET) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15_GYRO2_CS))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_PIN0_ADC_RSSI) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN1_ADC_CURRENT) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN2_ADC_VOLTAGE) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN3_GYRO2_INT) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN4_GYRO1_INT) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN5_NC) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN6_TX6) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN7_RX6) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN8_VBAT_SW) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN9_CAM_SEL) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN10_SPI_FLASH_SCK) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN11_SPI_FLASH_MISO) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN12_SPI_FLASH_MOSI) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN13_BEEPER) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN14_USB_DET) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN15_GYRO2_CS))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0_ADC_RSSI) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1_ADC_CURRENT) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2_ADC_VOLTAGE) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN3_GYRO2_INT) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN4_GYRO1_INT) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN5_NC) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6_TX6) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN7_RX6) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8_VBAT_SW) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9_CAM_SEL) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10_SPI_FLASH_SCK) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN11_SPI_FLASH_MISO) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN12_SPI_FLASH_MOSI) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN13_BEEPER) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14_USB_DET) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15_GYRO2_CS))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0_ADC_RSSI) |\
                                     PIN_ODR_HIGH(GPIOC_PIN1_ADC_CURRENT) |\
                                     PIN_ODR_HIGH(GPIOC_PIN2_ADC_VOLTAGE) |\
                                     PIN_ODR_HIGH(GPIOC_PIN3_GYRO2_INT) |\
                                     PIN_ODR_HIGH(GPIOC_PIN4_GYRO1_INT) |\
                                     PIN_ODR_HIGH(GPIOC_PIN5_NC) |\
                                     PIN_ODR_HIGH(GPIOC_PIN6_TX6) |\
                                     PIN_ODR_HIGH(GPIOC_PIN7_RX6) |\
                                     PIN_ODR_HIGH(GPIOC_PIN8_VBAT_SW) |\
                                     PIN_ODR_HIGH(GPIOC_PIN9_CAM_SEL) |\
                                     PIN_ODR_HIGH(GPIOC_PIN10_SPI_FLASH_SCK) |\
                                     PIN_ODR_HIGH(GPIOC_PIN11_SPI_FLASH_MISO) |\
                                     PIN_ODR_HIGH(GPIOC_PIN12_SPI_FLASH_MOSI) |\
                                     PIN_ODR_HIGH(GPIOC_PIN13_BEEPER) |\
                                     PIN_ODR_HIGH(GPIOC_PIN14_USB_DET) |\
                                     PIN_ODR_HIGH(GPIOC_PIN15_GYRO2_CS))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0_ADC_RSSI, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN1_ADC_CURRENT, 5U) |\
                                     PIN_AFIO_AF(GPIOC_PIN2_ADC_VOLTAGE, 5U) |\
                                     PIN_AFIO_AF(GPIOC_PIN3_GYRO2_INT, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN4_GYRO1_INT, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN5_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN6_TX6, 8U) |\
                                     PIN_AFIO_AF(GPIOC_PIN7_RX6, 8U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8_VBAT_SW, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN9_CAM_SEL, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN10_SPI_FLASH_SCK, 6U) |\
                                     PIN_AFIO_AF(GPIOC_PIN11_SPI_FLASH_MISO, 6U) |\
                                     PIN_AFIO_AF(GPIOC_PIN12_SPI_FLASH_MOSI, 6U) |\
                                     PIN_AFIO_AF(GPIOC_PIN13_BEEPER, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN14_USB_DET, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN15_GYRO2_CS, 0U))






#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN1_NC) |\
                                     PIN_MODE_OUTPUT(GPIOD_PIN2_FLASH_CS) |\
                                     PIN_MODE_INPUT(GPIOD_PIN3_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN4_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN5_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN6_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN7_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN8_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN9_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN10_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN11_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN12_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN13_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN14_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN15_NC))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2_FLASH_CS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15_NC))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_PIN0_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN1_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN2_FLASH_CS) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN3_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN4_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN5_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN6_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN7_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN8_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN9_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN10_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN11_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN12_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN13_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN14_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN15_NC))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1_NC) |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN2_FLASH_CS) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN3_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN4_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN5_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN6_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15_NC))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN1_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN2_FLASH_CS) |\
                                     PIN_ODR_HIGH(GPIOD_PIN3_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN4_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN5_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN6_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN7_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN8_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN9_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN10_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN11_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN12_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN13_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN14_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN15_NC))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN1_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN2_FLASH_CS, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN3_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN4_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN5_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN6_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN7_NC, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN9_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN10_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN11_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN12_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN13_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN14_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN15_NC, 0U))


#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |\
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |\
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |\
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |\
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |\
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U))
#define VAL_GPIOH_AFRH              (0)


#define VAL_GPIOE_MODER 0
#define VAL_GPIOE_OTYPER 0
#define VAL_GPIOE_OSPEEDR 0
#define VAL_GPIOE_PUPDR 0
#define VAL_GPIOE_ODR 0
#define VAL_GPIOE_AFRL 0
#define VAL_GPIOE_AFRH 0

#define VAL_GPIOF_MODER 0
#define VAL_GPIOF_OTYPER 0
#define VAL_GPIOF_OSPEEDR 0
#define VAL_GPIOF_PUPDR 0
#define VAL_GPIOF_ODR 0
#define VAL_GPIOF_AFRL 0
#define VAL_GPIOF_AFRH 0

#define VAL_GPIOG_MODER 0
#define VAL_GPIOG_OTYPER 0
#define VAL_GPIOG_OSPEEDR 0
#define VAL_GPIOG_PUPDR 0
#define VAL_GPIOG_ODR 0
#define VAL_GPIOG_AFRL 0
#define VAL_GPIOG_AFRH 0

#define VAL_GPIOI_MODER 0
#define VAL_GPIOI_OTYPER 0
#define VAL_GPIOI_OSPEEDR 0
#define VAL_GPIOI_PUPDR 0
#define VAL_GPIOI_ODR 0
#define VAL_GPIOI_AFRL 0
#define VAL_GPIOI_AFRH 0


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  bool arm_switch_is_armed(void);
  void arm_led_set(bool on);
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
