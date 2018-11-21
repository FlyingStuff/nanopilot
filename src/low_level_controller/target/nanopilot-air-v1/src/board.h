#ifndef BOARD_H
#define BOARD_H


/*
 * Board identifier.
 */
#define BOARD_NAME                  "Nanopilot-air v1"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#define STM32_LSECLK                0U

#define STM32_LSEDRV                (3U << 3U)

#define STM32_HSECLK                8000000U


/*
 * MCU type as defined in the ST header.
 */
#define STM32F303xE

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0_UART2_CTS        0U
#define GPIOA_PIN1_UART2_RTS        1U
#define GPIOA_PIN2_UART2_TX         2U
#define GPIOA_PIN3_UART2_RX         3U
#define GPIOA_PIN4_5V_ADC           4U
#define GPIOA_PIN5_VSERVO_ADC       5U
#define GPIOA_PIN6_I_BAT_ADC        6U
#define GPIOA_PIN7_V_BAT_ADC        7U
#define GPIOA_PIN8_NC               8U
#define GPIOA_PIN9_I2C_GPS_SCL      9U
#define GPIOA_PIN10_I2C_GPS_SDA     10U
#define GPIOA_PIN11_SERVO_9_PWM     11U
#define GPIOA_PIN12_SERVO_10_PWM    12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15_NC              15U

#define GPIOB_PIN0_SERVO_15_PWM     0U
#define GPIOB_PIN1_SERVO_16_PWM     1U
#define GPIOB_PIN2_NC               2U
#define GPIOB_SWO_NC                3U
#define GPIOB_PIN4_SERVO_13_PWM     4U
#define GPIOB_PIN5_SERVO_14_PWM     5U
#define GPIOB_PIN6_SENS_SCL         6U
#define GPIOB_PIN7_SENS_SDA         7U
#define GPIOB_PIN8_SERVO_11_PWM     8U
#define GPIOB_PIM9_SERVO_12_PWM     9U
#define GPIOB_PIN10_RC_TELEM_TX     10U
#define GPIOB_PIN11_RC_TELEM_RX     11U
#define GPIOB_PIN12_NC              12U
#define GPIOB_PIN13_SPI_SENS_SCLK   13U
#define GPIOB_PIN14_SPI_SENS_MISO   14U
#define GPIOB_PIN15_SPI_SENS_MOSI   15U

#define GPIOC_PIN0_SERVO_1_PWM      0U
#define GPIOC_PIN1_SERVO_2_PWM      1U
#define GPIOC_PIN2_SERVO_3_PWM      2U
#define GPIOC_PIN3_SERVO_4_PWM      3U
#define GPIOC_PIN4_DEBUG_TX         4U
#define GPIOC_PIN5_DEBUG_RX         5U
#define GPIOC_PIN6_SERVO_8_PWM      6U
#define GPIOC_PIN7_SERVO_7_PWM      7U
#define GPIOC_PIN8_SERVO_6_PWM      8U
#define GPIOC_PIN9_SERVO_5_PWM      9U
#define GPIOC_PIN10_GPS_TX          10U
#define GPIOC_PIN11_GPS_RX          11U
#define GPIOC_PIN12_NC              12U
#define GPIOC_PIN13_NC              13U
#define GPIOC_PIN14_NC              14U
#define GPIOC_PIN15_NC              15U

#define GPIOD_PIN0_NC               0U
#define GPIOD_PIN1_NC               1U
#define GPIOD_PIN2_RC_IN            2U
#define GPIOD_PIN3_NC               3U
#define GPIOD_PIN4_NC               4U
#define GPIOD_PIN5_NC               5U
#define GPIOD_PIN6_NC               6U
#define GPIOD_PIN7_NC               7U
#define GPIOD_PIN8_NC               8U
#define GPIOD_PIN9_TIME_SYNC_IN     9U
#define GPIOD_PIN10_SENS_LSM_CS     10U
#define GPIOD_PIN11_SENS_MPU_CS     11U
#define GPIOD_PIN12_SENS_MPU_INT    12U
#define GPIOD_PIN13_SENS_LSM_INT    13U
#define GPIOD_PIN14_SENS_H3LIS_INT  14U
#define GPIOD_PIN15_SENS_LIS3_INT   15U

#define GPIOE_PIN0_SENS_PWR_EN      0U
#define GPIOE_PIN1_NANOPI_PWR_EN    1U
#define GPIOE_PIN2_I2C_PWR_EN       2U
#define GPIOE_PIN3_USB2_PWR_EN      3U
#define GPIOE_PIN4_GPS_PWR_EN       4U
#define GPIOE_PIN5_TELEM_PWR_EN     5U
#define GPIOE_PIN6_USB3_PWR_EN      6U
#define GPIOE_PIN7_RESET_CONTROL_EN 7U
#define GPIOE_PIN8_HEARTBEAT_LED    8U
#define GPIOE_PIN9_ARM_LED          9U
#define GPIOE_PIN10_ARM_SWITCH      10U
#define GPIOE_PIN11_TP4             11U
#define GPIOE_PIN12_TP5             12U
#define GPIOE_PIN13_TP6             13U
#define GPIOE_PIN14_TP7             14U
#define GPIOE_PIN15_TP8             15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U
#define GPIOF_PIN2_NC               2U
#define GPIOF_PIN6_NC               6U
#define GPIOF_PIN9_NC               9U
#define GPIOF_PIN10_NC              10U



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
                                     PIN_MODE_ANALOG(GPIOA_PIN4_5V_ADC) |\
                                     PIN_MODE_ANALOG(GPIOA_PIN5_VSERVO_ADC) |\
                                     PIN_MODE_ANALOG(GPIOA_PIN6_I_BAT_ADC) |\
                                     PIN_MODE_ANALOG(GPIOA_PIN7_V_BAT_ADC) |\
                                     PIN_MODE_INPUT(GPIOA_PIN8_NC) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN9_I2C_GPS_SCL) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN10_I2C_GPS_SDA) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN11_SERVO_9_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOA_PIN12_SERVO_10_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |\
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |\
                                     PIN_MODE_INPUT(GPIOA_PIN15_NC))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2_UART2_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3_UART2_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4_5V_ADC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5_VSERVO_ADC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6_I_BAT_ADC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7_V_BAT_ADC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8_NC) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN9_I2C_GPS_SCL) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN10_I2C_GPS_SDA) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN11_SERVO_9_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN12_SERVO_10_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15_NC))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN2_UART2_TX) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN3_UART2_RX) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN4_5V_ADC) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN5_VSERVO_ADC) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN6_I_BAT_ADC) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN7_V_BAT_ADC) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN8_NC) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN9_I2C_GPS_SCL) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN10_I2C_GPS_SDA) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN11_SERVO_9_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN12_SERVO_10_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |\
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |\
                                     PIN_OSPEED_HIGH(GPIOA_PIN15_NC))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN2_UART2_TX) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN3_UART2_RX) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4_5V_ADC) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5_VSERVO_ADC) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6_I_BAT_ADC) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN7_V_BAT_ADC) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN8_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN9_I2C_GPS_SCL) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN10_I2C_GPS_SDA) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN11_SERVO_9_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOA_PIN12_SERVO_10_PWM) |\
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |\
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15_NC))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0_UART2_CTS) |\
                                     PIN_ODR_HIGH(GPIOA_PIN1_UART2_RTS) |\
                                     PIN_ODR_HIGH(GPIOA_PIN2_UART2_TX) |\
                                     PIN_ODR_HIGH(GPIOA_PIN3_UART2_RX) |\
                                     PIN_ODR_HIGH(GPIOA_PIN4_5V_ADC) |\
                                     PIN_ODR_LOW(GPIOA_PIN5_VSERVO_ADC) |\
                                     PIN_ODR_HIGH(GPIOA_PIN6_I_BAT_ADC) |\
                                     PIN_ODR_HIGH(GPIOA_PIN7_V_BAT_ADC) |\
                                     PIN_ODR_HIGH(GPIOA_PIN8_NC) |\
                                     PIN_ODR_HIGH(GPIOA_PIN9_I2C_GPS_SCL) |\
                                     PIN_ODR_HIGH(GPIOA_PIN10_I2C_GPS_SDA) |\
                                     PIN_ODR_HIGH(GPIOA_PIN11_SERVO_9_PWM) |\
                                     PIN_ODR_HIGH(GPIOA_PIN12_SERVO_10_PWM) |\
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |\
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |\
                                     PIN_ODR_HIGH(GPIOA_PIN15_NC))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0_UART2_CTS, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN1_UART2_RTS, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN2_UART2_TX, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN3_UART2_RX, 7U) |\
                                     PIN_AFIO_AF(GPIOA_PIN4_5V_ADC, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN5_VSERVO_ADC, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN6_I_BAT_ADC, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN7_V_BAT_ADC, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN9_I2C_GPS_SCL, 4U) |\
                                     PIN_AFIO_AF(GPIOA_PIN10_I2C_GPS_SDA, 4U) |\
                                     PIN_AFIO_AF(GPIOA_PIN11_SERVO_9_PWM, 10U) |\
                                     PIN_AFIO_AF(GPIOA_PIN12_SERVO_10_PWM, 10U) |\
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |\
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |\
                                     PIN_AFIO_AF(GPIOA_PIN15_NC, 0U))





#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_PIN0_SERVO_15_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN1_SERVO_16_PWM) |\
                                     PIN_MODE_INPUT(GPIOB_PIN2_NC) |\
                                     PIN_MODE_INPUT(GPIOB_SWO_NC) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN4_SERVO_13_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN5_SERVO_14_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN6_SENS_SCL) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN7_SENS_SDA) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN8_SERVO_11_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIM9_SERVO_12_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN10_RC_TELEM_TX) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN11_RC_TELEM_RX) |\
                                     PIN_MODE_INPUT(GPIOB_PIN12_NC) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN13_SPI_SENS_SCLK) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN14_SPI_SENS_MISO) |\
                                     PIN_MODE_ALTERNATE(GPIOB_PIN15_SPI_SENS_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0_SERVO_15_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1_SERVO_16_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4_SERVO_13_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5_SERVO_14_PWM) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN6_SENS_SCL) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN7_SENS_SDA) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8_SERVO_11_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIM9_SERVO_12_PWM) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_PIN10_RC_TELEM_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11_RC_TELEM_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13_SPI_SENS_SCLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14_SPI_SENS_MISO) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15_SPI_SENS_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_PIN0_SERVO_15_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN1_SERVO_16_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN2_NC) |\
                                     PIN_OSPEED_HIGH(GPIOB_SWO_NC) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN4_SERVO_13_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN5_SERVO_14_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN6_SENS_SCL) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN7_SENS_SDA) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN8_SERVO_11_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIM9_SERVO_12_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN10_RC_TELEM_TX) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN11_RC_TELEM_RX) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN12_NC) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN13_SPI_SENS_SCLK) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN14_SPI_SENS_MISO) |\
                                     PIN_OSPEED_HIGH(GPIOB_PIN15_SPI_SENS_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_PIN0_SERVO_15_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1_SERVO_16_PWM) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOB_SWO_NC) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4_SERVO_13_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5_SERVO_14_PWM) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN6_SENS_SCL) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN7_SENS_SDA) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8_SERVO_11_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIM9_SERVO_12_PWM) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN10_RC_TELEM_TX) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN11_RC_TELEM_RX) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN12_NC) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13_SPI_SENS_SCLK) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN14_SPI_SENS_MISO) |\
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15_SPI_SENS_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_PIN0_SERVO_15_PWM) |\
                                     PIN_ODR_HIGH(GPIOB_PIN1_SERVO_16_PWM) |\
                                     PIN_ODR_HIGH(GPIOB_PIN2_NC) |\
                                     PIN_ODR_HIGH(GPIOB_SWO_NC) |\
                                     PIN_ODR_HIGH(GPIOB_PIN4_SERVO_13_PWM) |\
                                     PIN_ODR_HIGH(GPIOB_PIN5_SERVO_14_PWM) |\
                                     PIN_ODR_HIGH(GPIOB_PIN6_SENS_SCL) |\
                                     PIN_ODR_HIGH(GPIOB_PIN7_SENS_SDA) |\
                                     PIN_ODR_HIGH(GPIOB_PIN8_SERVO_11_PWM) |\
                                     PIN_ODR_HIGH(GPIOB_PIM9_SERVO_12_PWM) |\
                                     PIN_ODR_HIGH(GPIOB_PIN10_RC_TELEM_TX) |\
                                     PIN_ODR_HIGH(GPIOB_PIN11_RC_TELEM_RX) |\
                                     PIN_ODR_HIGH(GPIOB_PIN12_NC) |\
                                     PIN_ODR_HIGH(GPIOB_PIN13_SPI_SENS_SCLK) |\
                                     PIN_ODR_HIGH(GPIOB_PIN14_SPI_SENS_MISO) |\
                                     PIN_ODR_HIGH(GPIOB_PIN15_SPI_SENS_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0_SERVO_15_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN1_SERVO_16_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN2_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOB_SWO_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOB_PIN4_SERVO_13_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN5_SERVO_14_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN6_SENS_SCL, 4U) |\
                                     PIN_AFIO_AF(GPIOB_PIN7_SENS_SDA, 4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8_SERVO_11_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIM9_SERVO_12_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN10_RC_TELEM_TX, 7U) |\
                                     PIN_AFIO_AF(GPIOB_PIN11_RC_TELEM_RX, 7U) |\
                                     PIN_AFIO_AF(GPIOB_PIN12_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOB_PIN13_SPI_SENS_SCLK, 5U) |\
                                     PIN_AFIO_AF(GPIOB_PIN14_SPI_SENS_MISO, 5U) |\
                                     PIN_AFIO_AF(GPIOB_PIN15_SPI_SENS_MOSI, 5U))





#define VAL_GPIOC_MODER             (PIN_MODE_ALTERNATE(GPIOC_PIN0_SERVO_1_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN1_SERVO_2_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN2_SERVO_3_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN3_SERVO_4_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN4_DEBUG_TX) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN5_DEBUG_RX) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN6_SERVO_8_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN7_SERVO_7_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN8_SERVO_6_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN9_SERVO_5_PWM) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN10_GPS_TX) |\
                                     PIN_MODE_ALTERNATE(GPIOC_PIN11_GPS_RX) |\
                                     PIN_MODE_INPUT(GPIOC_PIN12_NC) |\
                                     PIN_MODE_INPUT(GPIOC_PIN13_NC) |\
                                     PIN_MODE_INPUT(GPIOC_PIN14_NC) |\
                                     PIN_MODE_INPUT(GPIOC_PIN15_NC))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0_SERVO_1_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1_SERVO_2_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2_SERVO_3_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3_SERVO_4_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4_DEBUG_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5_DEBUG_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6_SERVO_8_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7_SERVO_7_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8_SERVO_6_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9_SERVO_5_PWM) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10_GPS_TX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11_GPS_RX) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15_NC))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_PIN0_SERVO_1_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN1_SERVO_2_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN2_SERVO_3_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN3_SERVO_4_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN4_DEBUG_TX) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN5_DEBUG_RX) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN6_SERVO_8_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN7_SERVO_7_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN8_SERVO_6_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN9_SERVO_5_PWM) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN10_GPS_TX) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN11_GPS_RX) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN12_NC) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN13_NC) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN14_NC) |\
                                     PIN_OSPEED_HIGH(GPIOC_PIN15_NC))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0_SERVO_1_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1_SERVO_2_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2_SERVO_3_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3_SERVO_4_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN4_DEBUG_TX) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN5_DEBUG_RX) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6_SERVO_8_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN7_SERVO_7_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8_SERVO_6_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9_SERVO_5_PWM) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10_GPS_TX) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN11_GPS_RX) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN12_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN13_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN14_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOC_PIN15_NC))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0_SERVO_1_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN1_SERVO_2_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN2_SERVO_3_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN3_SERVO_4_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN4_DEBUG_TX) |\
                                     PIN_ODR_HIGH(GPIOC_PIN5_DEBUG_RX) |\
                                     PIN_ODR_HIGH(GPIOC_PIN6_SERVO_8_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN7_SERVO_7_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN8_SERVO_6_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN9_SERVO_5_PWM) |\
                                     PIN_ODR_HIGH(GPIOC_PIN10_GPS_TX) |\
                                     PIN_ODR_HIGH(GPIOC_PIN11_GPS_RX) |\
                                     PIN_ODR_HIGH(GPIOC_PIN12_NC) |\
                                     PIN_ODR_HIGH(GPIOC_PIN13_NC) |\
                                     PIN_ODR_HIGH(GPIOC_PIN14_NC) |\
                                     PIN_ODR_HIGH(GPIOC_PIN15_NC))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0_SERVO_1_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOC_PIN1_SERVO_2_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOC_PIN2_SERVO_3_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOC_PIN3_SERVO_4_PWM, 2U) |\
                                     PIN_AFIO_AF(GPIOC_PIN4_DEBUG_TX, 7U) |\
                                     PIN_AFIO_AF(GPIOC_PIN5_DEBUG_RX, 7U) |\
                                     PIN_AFIO_AF(GPIOC_PIN6_SERVO_8_PWM, 4U) |\
                                     PIN_AFIO_AF(GPIOC_PIN7_SERVO_7_PWM, 4U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8_SERVO_6_PWM, 4U) |\
                                     PIN_AFIO_AF(GPIOC_PIN9_SERVO_5_PWM, 4U) |\
                                     PIN_AFIO_AF(GPIOC_PIN10_GPS_TX, 5U) |\
                                     PIN_AFIO_AF(GPIOC_PIN11_GPS_RX, 5U) |\
                                     PIN_AFIO_AF(GPIOC_PIN12_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN13_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN14_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOC_PIN15_NC, 0U))






#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN1_NC) |\
                                     PIN_MODE_ALTERNATE(GPIOD_PIN2_RC_IN) |\
                                     PIN_MODE_INPUT(GPIOD_PIN3_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN4_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN5_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN6_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN7_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN8_NC) |\
                                     PIN_MODE_INPUT(GPIOD_PIN9_TIME_SYNC_IN) |\
                                     PIN_MODE_OUTPUT(GPIOD_PIN10_SENS_LSM_CS) |\
                                     PIN_MODE_OUTPUT(GPIOD_PIN11_SENS_MPU_CS) |\
                                     PIN_MODE_INPUT(GPIOD_PIN12_SENS_MPU_INT) |\
                                     PIN_MODE_INPUT(GPIOD_PIN13_SENS_LSM_INT) |\
                                     PIN_MODE_INPUT(GPIOD_PIN14_SENS_H3LIS_INT) |\
                                     PIN_MODE_INPUT(GPIOD_PIN15_SENS_LIS3_INT))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2_RC_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9_TIME_SYNC_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10_SENS_LSM_CS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11_SENS_MPU_CS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12_SENS_MPU_INT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13_SENS_LSM_INT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14_SENS_H3LIS_INT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15_SENS_LIS3_INT))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_PIN0_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN1_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN2_RC_IN) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN3_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN4_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN5_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN6_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN7_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN8_NC) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN9_TIME_SYNC_IN) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN10_SENS_LSM_CS) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN11_SENS_MPU_CS) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN12_SENS_MPU_INT) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN13_SENS_LSM_INT) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN14_SENS_H3LIS_INT) |\
                                     PIN_OSPEED_HIGH(GPIOD_PIN15_SENS_LIS3_INT))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN2_RC_IN) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN3_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN4_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN5_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN6_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9_TIME_SYNC_IN) |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10_SENS_LSM_CS) |\
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11_SENS_MPU_CS) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12_SENS_MPU_INT) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13_SENS_LSM_INT) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14_SENS_H3LIS_INT) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15_SENS_LIS3_INT))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN1_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN2_RC_IN) |\
                                     PIN_ODR_HIGH(GPIOD_PIN3_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN4_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN5_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN6_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN7_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN8_NC) |\
                                     PIN_ODR_HIGH(GPIOD_PIN9_TIME_SYNC_IN) |\
                                     PIN_ODR_HIGH(GPIOD_PIN10_SENS_LSM_CS) |\
                                     PIN_ODR_HIGH(GPIOD_PIN11_SENS_MPU_CS) |\
                                     PIN_ODR_HIGH(GPIOD_PIN12_SENS_MPU_INT) |\
                                     PIN_ODR_HIGH(GPIOD_PIN13_SENS_LSM_INT) |\
                                     PIN_ODR_HIGH(GPIOD_PIN14_SENS_H3LIS_INT) |\
                                     PIN_ODR_HIGH(GPIOD_PIN15_SENS_LIS3_INT))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN1_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN2_RC_IN, 5U) |\
                                     PIN_AFIO_AF(GPIOD_PIN3_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN4_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN5_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN6_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN7_NC, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN9_TIME_SYNC_IN, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN10_SENS_LSM_CS, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN11_SENS_MPU_CS, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN12_SENS_MPU_INT, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN13_SENS_LSM_INT, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN14_SENS_H3LIS_INT, 0U) |\
                                     PIN_AFIO_AF(GPIOD_PIN15_SENS_LIS3_INT, 0U))






#define VAL_GPIOE_MODER             (PIN_MODE_OUTPUT(GPIOE_PIN0_SENS_PWR_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN1_NANOPI_PWR_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN2_I2C_PWR_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN3_USB2_PWR_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN4_GPS_PWR_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN5_TELEM_PWR_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN6_USB3_PWR_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN7_RESET_CONTROL_EN) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN8_HEARTBEAT_LED) |\
                                     PIN_MODE_OUTPUT(GPIOE_PIN9_ARM_LED) |\
                                     PIN_MODE_INPUT(GPIOE_PIN10_ARM_SWITCH) |\
                                     PIN_MODE_INPUT(GPIOE_PIN11_TP4) |\
                                     PIN_MODE_INPUT(GPIOE_PIN12_TP5) |\
                                     PIN_MODE_INPUT(GPIOE_PIN13_TP6) |\
                                     PIN_MODE_INPUT(GPIOE_PIN14_TP7) |\
                                     PIN_MODE_INPUT(GPIOE_PIN15_TP8))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0_SENS_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1_NANOPI_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2_I2C_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3_USB2_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4_GPS_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5_TELEM_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6_USB3_PWR_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7_RESET_CONTROL_EN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8_HEARTBEAT_LED) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9_ARM_LED) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10_ARM_SWITCH) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11_TP4) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12_TP5) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13_TP6) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14_TP7) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15_TP8))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_PIN0_SENS_PWR_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN1_NANOPI_PWR_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN2_I2C_PWR_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN3_USB2_PWR_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN4_GPS_PWR_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN5_TELEM_PWR_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN6_USB3_PWR_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN7_RESET_CONTROL_EN) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN8_HEARTBEAT_LED) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN9_ARM_LED) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN10_ARM_SWITCH) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN11_TP4) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN12_TP5) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN13_TP6) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN14_TP7) |\
                                     PIN_OSPEED_HIGH(GPIOE_PIN15_TP8))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0_SENS_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1_NANOPI_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2_I2C_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3_USB2_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4_GPS_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5_TELEM_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6_USB3_PWR_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7_RESET_CONTROL_EN) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8_HEARTBEAT_LED) |\
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9_ARM_LED) |\
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10_ARM_SWITCH) |\
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11_TP4) |\
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12_TP5) |\
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13_TP6) |\
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14_TP7) |\
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15_TP8))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0_SENS_PWR_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN1_NANOPI_PWR_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN2_I2C_PWR_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN3_USB2_PWR_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN4_GPS_PWR_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN5_TELEM_PWR_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN6_USB3_PWR_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN7_RESET_CONTROL_EN) |\
                                     PIN_ODR_HIGH(GPIOE_PIN8_HEARTBEAT_LED) |\
                                     PIN_ODR_HIGH(GPIOE_PIN9_ARM_LED) |\
                                     PIN_ODR_HIGH(GPIOE_PIN10_ARM_SWITCH) |\
                                     PIN_ODR_HIGH(GPIOE_PIN11_TP4) |\
                                     PIN_ODR_HIGH(GPIOE_PIN12_TP5) |\
                                     PIN_ODR_HIGH(GPIOE_PIN13_TP6) |\
                                     PIN_ODR_HIGH(GPIOE_PIN14_TP7) |\
                                     PIN_ODR_HIGH(GPIOE_PIN15_TP8))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0_SENS_PWR_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN1_NANOPI_PWR_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN2_I2C_PWR_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN3_USB2_PWR_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN4_GPS_PWR_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN5_TELEM_PWR_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN6_USB3_PWR_EN, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN7_RESET_CONTROL_EN, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8_HEARTBEAT_LED, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN9_ARM_LED, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN10_ARM_SWITCH, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN11_TP4, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN12_TP5, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN13_TP6, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN14_TP7, 0U) |\
                                     PIN_AFIO_AF(GPIOE_PIN15_TP8, 0U))







#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |\
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT) |\
                                     PIN_MODE_INPUT(GPIOF_PIN2_NC) |\
                                     PIN_MODE_INPUT(GPIOF_PIN6_NC) |\
                                     PIN_MODE_INPUT(GPIOF_PIN9_NC) |\
                                     PIN_MODE_INPUT(GPIOF_PIN10_NC))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9_NC) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10_NC))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_OSC_IN) |\
                                     PIN_OSPEED_HIGH(GPIOF_OSC_OUT) |\
                                     PIN_OSPEED_HIGH(GPIOF_PIN2_NC) |\
                                     PIN_OSPEED_HIGH(GPIOF_PIN6_NC) |\
                                     PIN_OSPEED_HIGH(GPIOF_PIN9_NC) |\
                                     PIN_OSPEED_HIGH(GPIOF_PIN10_NC))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |\
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT) |\
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOF_PIN6_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOF_PIN9_NC) |\
                                     PIN_PUPDR_PULLUP(GPIOF_PIN10_NC))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_OSC_IN) |\
                                     PIN_ODR_HIGH(GPIOF_OSC_OUT) |\
                                     PIN_ODR_HIGH(GPIOF_PIN2_NC) |\
                                     PIN_ODR_HIGH(GPIOF_PIN6_NC) |\
                                     PIN_ODR_HIGH(GPIOF_PIN9_NC) |\
                                     PIN_ODR_HIGH(GPIOF_PIN10_NC))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0U) |\
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0U) |\
                                     PIN_AFIO_AF(GPIOF_PIN2_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOF_PIN6_NC, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN9_NC, 0U) |\
                                     PIN_AFIO_AF(GPIOF_PIN10_NC, 0U))


#define VAL_GPIOG_MODER 0
#define VAL_GPIOG_OTYPER 0
#define VAL_GPIOG_OSPEEDR 0
#define VAL_GPIOG_PUPDR 0
#define VAL_GPIOG_ODR 0
#define VAL_GPIOG_AFRL 0
#define VAL_GPIOG_AFRH 0

#define VAL_GPIOH_MODER 0
#define VAL_GPIOH_OTYPER 0
#define VAL_GPIOH_OSPEEDR 0
#define VAL_GPIOH_PUPDR 0
#define VAL_GPIOH_ODR 0
#define VAL_GPIOH_AFRL 0
#define VAL_GPIOH_AFRH 0

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
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
