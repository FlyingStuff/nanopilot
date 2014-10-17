#ifndef _BOARD_H_
#define _BOARD_H_

#define BOARD_INS
#define BOARD_NAME                  "Spieler Brothers - INS Board"


#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#define STM32_HSECLK                16000000

#define STM32_VDD                   330

#define STM32F405xx

/*
 * IO pins assignments.
 */
#define GPIOA_UART4_TX_CONN2        0
#define GPIOA_UART4_RX_CONN2        1
#define GPIOA_UART2_TX_CONN3        2
#define GPIOA_UART2_RX_CONN3        3
#define GPIOA_NC_4                  4
#define GPIOA_SPI1_SCK              5
#define GPIOA_SPI1_MISO             6
#define GPIOA_SPI1_MOSI             7
#define GPIOA_LED_HEARTBEAT         8
#define GPIOA_USB_VBUS              9
#define GPIOA_LED_ERROR             10
#define GPIOA_USB_DM                11
#define GPIOA_USB_DP                12
#define GPIOA_JTAG_TMS              13
#define GPIOA_JTAG_TCK              14
#define GPIOA_JTAG_TDI              15

#define GPIOB_HMC5883L_DRDY         0
#define GPIOB_NC_1                  1
#define GPIOB_NC_2                  2
#define GPIOB_JTAG_TDO              3
#define GPIOB_JTAG_TRST             4
#define GPIOB_IO_POWER_EN           5
#define GPIOB_UART1_TX_CONN1        6
#define GPIOB_UART1_RX_CONN1        7
#define GPIOB_I2C1_SCL              8
#define GPIOB_I2C1_SDA              9
#define GPIOB_I2C2_SCL_CONN         10
#define GPIOB_I2C2_SDA_CONN         11
#define GPIOB_CAN2_RX_CONN          12
#define GPIOB_CAN2_TX_CONN          13
#define GPIOB_LED_STATUS            14
#define GPIOB_LED_SDCARD            15

#define GPIOC_VBAT_MON_AIN          0
#define GPIOC_SDCARD_DETECT         1
#define GPIOC_MPU6000_INT           2
#define GPIOC_MPU6000_FSYNC         3
#define GPIOC_MPU6000_CS            4
#define GPIOC_CAN_CONN_EN           5
#define GPIOC_UART6_TX_CONN4        6
#define GPIOC_UART6_RX_CONN4        7
#define GPIOC_SDIO_D0               8
#define GPIOC_SDIO_D1               9
#define GPIOC_SDIO_D2               10
#define GPIOC_SDIO_D3               11
#define GPIOC_SDIO_CK               12
#define GPIOC_SDCARD_POWER_EN       13
#define GPIOC_VCC_A_POWER_EN        14
#define GPIOC_H3LIS331DL_INT        15

#define GPIOD_SDIO_CMD              2

#define GPIOH_OSC_IN                0
#define GPIOH_OSC_OUT               1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))


// GPIO A

#define VAL_GPIOA_MODER     ( PIN_MODE_ALTERNATE( GPIOA_UART4_TX_CONN2  )       \
                            | PIN_MODE_ALTERNATE( GPIOA_UART4_RX_CONN2  )       \
                            | PIN_MODE_ALTERNATE( GPIOA_UART2_TX_CONN3  )       \
                            | PIN_MODE_ALTERNATE( GPIOA_UART2_RX_CONN3  )       \
                            | PIN_MODE_INPUT(     GPIOA_NC_4            )       \
                            | PIN_MODE_INPUT(     GPIOA_SPI1_SCK        )       \
                            | PIN_MODE_INPUT(     GPIOA_SPI1_MISO       )       \
                            | PIN_MODE_INPUT(     GPIOA_SPI1_MOSI       )       \
                            | PIN_MODE_OUTPUT(    GPIOA_LED_HEARTBEAT   )       \
                            | PIN_MODE_INPUT(     GPIOA_USB_VBUS        )       \
                            | PIN_MODE_OUTPUT(    GPIOA_LED_ERROR       )       \
                            | PIN_MODE_ALTERNATE( GPIOA_USB_DM          )       \
                            | PIN_MODE_ALTERNATE( GPIOA_USB_DP          )       \
                            | PIN_MODE_ALTERNATE( GPIOA_JTAG_TMS        )       \
                            | PIN_MODE_ALTERNATE( GPIOA_JTAG_TCK        )       \
                            | PIN_MODE_ALTERNATE( GPIOA_JTAG_TDI        ) )


#define VAL_GPIOA_OTYPER    ( PIN_OTYPE_PUSHPULL( GPIOA_UART4_TX_CONN2  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_UART4_RX_CONN2  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_UART2_TX_CONN3  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_UART2_RX_CONN3  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_NC_4            )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_SPI1_SCK        )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_SPI1_MISO       )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_SPI1_MOSI       )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_LED_HEARTBEAT   )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_USB_VBUS        )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_LED_ERROR       )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_USB_DM          )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_USB_DP          )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_JTAG_TMS        )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_JTAG_TCK        )   \
                            | PIN_OTYPE_PUSHPULL( GPIOA_JTAG_TDI        ) )

#define VAL_GPIOA_OSPEEDR   ( PIN_OSPEED_100M( GPIOA_UART4_TX_CONN2  )      \
                            | PIN_OSPEED_100M( GPIOA_UART4_RX_CONN2  )      \
                            | PIN_OSPEED_100M( GPIOA_UART2_TX_CONN3  )      \
                            | PIN_OSPEED_100M( GPIOA_UART2_RX_CONN3  )      \
                            | PIN_OSPEED_100M( GPIOA_NC_4            )      \
                            | PIN_OSPEED_100M( GPIOA_SPI1_SCK        )      \
                            | PIN_OSPEED_100M( GPIOA_SPI1_MISO       )      \
                            | PIN_OSPEED_100M( GPIOA_SPI1_MOSI       )      \
                            | PIN_OSPEED_100M( GPIOA_LED_HEARTBEAT   )      \
                            | PIN_OSPEED_100M( GPIOA_USB_VBUS        )      \
                            | PIN_OSPEED_100M( GPIOA_LED_ERROR       )      \
                            | PIN_OSPEED_100M( GPIOA_USB_DM          )      \
                            | PIN_OSPEED_100M( GPIOA_USB_DP          )      \
                            | PIN_OSPEED_100M( GPIOA_JTAG_TMS        )      \
                            | PIN_OSPEED_100M( GPIOA_JTAG_TCK        )      \
                            | PIN_OSPEED_100M( GPIOA_JTAG_TDI        ) )

#define VAL_GPIOA_PUPDR     ( PIN_PUPDR_FLOATING( GPIOA_UART4_TX_CONN2  )   \
                            | PIN_PUPDR_PULLUP(   GPIOA_UART4_RX_CONN2  )   \
                            | PIN_PUPDR_FLOATING( GPIOA_UART2_TX_CONN3  )   \
                            | PIN_PUPDR_PULLUP(   GPIOA_UART2_RX_CONN3  )   \
                            | PIN_PUPDR_PULLDOWN( GPIOA_NC_4            )   \
                            | PIN_PUPDR_FLOATING( GPIOA_SPI1_SCK        )   \
                            | PIN_PUPDR_FLOATING( GPIOA_SPI1_MISO       )   \
                            | PIN_PUPDR_FLOATING( GPIOA_SPI1_MOSI       )   \
                            | PIN_PUPDR_FLOATING( GPIOA_LED_HEARTBEAT   )   \
                            | PIN_PUPDR_FLOATING( GPIOA_USB_VBUS        )   \
                            | PIN_PUPDR_FLOATING( GPIOA_LED_ERROR       )   \
                            | PIN_PUPDR_FLOATING( GPIOA_USB_DM          )   \
                            | PIN_PUPDR_FLOATING( GPIOA_USB_DP          )   \
                            | PIN_PUPDR_PULLUP(   GPIOA_JTAG_TMS        )   \
                            | PIN_PUPDR_PULLDOWN( GPIOA_JTAG_TCK        )   \
                            | PIN_PUPDR_PULLUP(   GPIOA_JTAG_TDI        ) )

#define VAL_GPIOA_ODR       ( PIN_ODR_LOW( GPIOA_UART4_TX_CONN2  )          \
                            | PIN_ODR_LOW( GPIOA_UART4_RX_CONN2  )          \
                            | PIN_ODR_LOW( GPIOA_UART2_TX_CONN3  )          \
                            | PIN_ODR_LOW( GPIOA_UART2_RX_CONN3  )          \
                            | PIN_ODR_LOW( GPIOA_NC_4            )          \
                            | PIN_ODR_LOW( GPIOA_SPI1_SCK        )          \
                            | PIN_ODR_LOW( GPIOA_SPI1_MISO       )          \
                            | PIN_ODR_LOW( GPIOA_SPI1_MOSI       )          \
                            | PIN_ODR_LOW( GPIOA_LED_HEARTBEAT   )          \
                            | PIN_ODR_LOW( GPIOA_USB_VBUS        )          \
                            | PIN_ODR_LOW( GPIOA_LED_ERROR       )          \
                            | PIN_ODR_LOW( GPIOA_USB_DM          )          \
                            | PIN_ODR_LOW( GPIOA_USB_DP          )          \
                            | PIN_ODR_LOW( GPIOA_JTAG_TMS        )          \
                            | PIN_ODR_LOW( GPIOA_JTAG_TCK        )          \
                            | PIN_ODR_LOW( GPIOA_JTAG_TDI        ) )

#define VAL_GPIOA_AFRL      ( PIN_AFIO_AF( GPIOA_UART4_TX_CONN2  ,  8)      \
                            | PIN_AFIO_AF( GPIOA_UART4_RX_CONN2  ,  8)      \
                            | PIN_AFIO_AF( GPIOA_UART2_TX_CONN3  ,  7)      \
                            | PIN_AFIO_AF( GPIOA_UART2_RX_CONN3  ,  7)      \
                            | PIN_AFIO_AF( GPIOA_NC_4            ,  0)      \
                            | PIN_AFIO_AF( GPIOA_SPI1_SCK        ,  0)      \
                            | PIN_AFIO_AF( GPIOA_SPI1_MISO       ,  0)      \
                            | PIN_AFIO_AF( GPIOA_SPI1_MOSI       ,  0) )

#define VAL_GPIOA_AFRH      ( PIN_AFIO_AF( GPIOA_LED_HEARTBEAT   ,  0)      \
                            | PIN_AFIO_AF( GPIOA_USB_VBUS        ,  0)      \
                            | PIN_AFIO_AF( GPIOA_LED_ERROR       ,  0)      \
                            | PIN_AFIO_AF( GPIOA_USB_DM          , 10)      \
                            | PIN_AFIO_AF( GPIOA_USB_DP          , 10)      \
                            | PIN_AFIO_AF( GPIOA_JTAG_TMS        ,  0)      \
                            | PIN_AFIO_AF( GPIOA_JTAG_TCK        ,  0)      \
                            | PIN_AFIO_AF( GPIOA_JTAG_TDI        ,  0) )


// GPIO B

#define VAL_GPIOB_MODER     ( PIN_MODE_INPUT(     GPIOB_HMC5883L_DRDY  )       \
                            | PIN_MODE_INPUT(     GPIOB_NC_1           )       \
                            | PIN_MODE_INPUT(     GPIOB_NC_2           )       \
                            | PIN_MODE_ALTERNATE( GPIOB_JTAG_TDO       )       \
                            | PIN_MODE_ALTERNATE( GPIOB_JTAG_TRST      )       \
                            | PIN_MODE_INPUT(     GPIOB_IO_POWER_EN    )       \
                            | PIN_MODE_ALTERNATE( GPIOB_UART1_TX_CONN1 )       \
                            | PIN_MODE_ALTERNATE( GPIOB_UART1_RX_CONN1 )       \
                            | PIN_MODE_INPUT(     GPIOB_I2C1_SCL       )       \
                            | PIN_MODE_INPUT(     GPIOB_I2C1_SDA       )       \
                            | PIN_MODE_INPUT(     GPIOB_I2C2_SCL_CONN  )       \
                            | PIN_MODE_INPUT(     GPIOB_I2C2_SDA_CONN  )       \
                            | PIN_MODE_INPUT(     GPIOB_CAN2_RX_CONN   )       \
                            | PIN_MODE_INPUT(     GPIOB_CAN2_TX_CONN   )       \
                            | PIN_MODE_OUTPUT(    GPIOB_LED_STATUS     )       \
                            | PIN_MODE_OUTPUT(    GPIOB_LED_SDCARD     ) )


#define VAL_GPIOB_OTYPER    ( PIN_OTYPE_PUSHPULL( GPIOB_HMC5883L_DRDY  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_NC_1           )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_NC_2           )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_JTAG_TDO       )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_JTAG_TRST      )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_IO_POWER_EN    )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_UART1_TX_CONN1 )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_UART1_RX_CONN1 )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_I2C1_SCL       )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_I2C1_SDA       )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_I2C2_SCL_CONN  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_I2C2_SDA_CONN  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_CAN2_RX_CONN   )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_CAN2_TX_CONN   )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_LED_STATUS     )   \
                            | PIN_OTYPE_PUSHPULL( GPIOB_LED_SDCARD     ) )

#define VAL_GPIOB_OSPEEDR   ( PIN_OSPEED_100M( GPIOB_HMC5883L_DRDY  )      \
                            | PIN_OSPEED_100M( GPIOB_NC_1           )      \
                            | PIN_OSPEED_100M( GPIOB_NC_2           )      \
                            | PIN_OSPEED_100M( GPIOB_JTAG_TDO       )      \
                            | PIN_OSPEED_100M( GPIOB_JTAG_TRST      )      \
                            | PIN_OSPEED_100M( GPIOB_IO_POWER_EN    )      \
                            | PIN_OSPEED_100M( GPIOB_UART1_TX_CONN1 )      \
                            | PIN_OSPEED_100M( GPIOB_UART1_RX_CONN1 )      \
                            | PIN_OSPEED_100M( GPIOB_I2C1_SCL       )      \
                            | PIN_OSPEED_100M( GPIOB_I2C1_SDA       )      \
                            | PIN_OSPEED_100M( GPIOB_I2C2_SCL_CONN  )      \
                            | PIN_OSPEED_100M( GPIOB_I2C2_SDA_CONN  )      \
                            | PIN_OSPEED_100M( GPIOB_CAN2_RX_CONN   )      \
                            | PIN_OSPEED_100M( GPIOB_CAN2_TX_CONN   )      \
                            | PIN_OSPEED_100M( GPIOB_LED_STATUS     )      \
                            | PIN_OSPEED_100M( GPIOB_LED_SDCARD     ) )

#define VAL_GPIOB_PUPDR     ( PIN_PUPDR_FLOATING( GPIOB_HMC5883L_DRDY  )   \
                            | PIN_PUPDR_PULLDOWN( GPIOB_NC_1           )   \
                            | PIN_PUPDR_PULLDOWN( GPIOB_NC_2           )   \
                            | PIN_PUPDR_FLOATING( GPIOB_JTAG_TDO       )   \
                            | PIN_PUPDR_PULLUP(   GPIOB_JTAG_TRST      )   \
                            | PIN_PUPDR_FLOATING( GPIOB_IO_POWER_EN    )   \
                            | PIN_PUPDR_FLOATING( GPIOB_UART1_TX_CONN1 )   \
                            | PIN_PUPDR_PULLUP(   GPIOB_UART1_RX_CONN1 )   \
                            | PIN_PUPDR_FLOATING( GPIOB_I2C1_SCL       )   \
                            | PIN_PUPDR_FLOATING( GPIOB_I2C1_SDA       )   \
                            | PIN_PUPDR_FLOATING( GPIOB_I2C2_SCL_CONN  )   \
                            | PIN_PUPDR_FLOATING( GPIOB_I2C2_SDA_CONN  )   \
                            | PIN_PUPDR_FLOATING( GPIOB_CAN2_RX_CONN   )   \
                            | PIN_PUPDR_FLOATING( GPIOB_CAN2_TX_CONN   )   \
                            | PIN_PUPDR_FLOATING( GPIOB_LED_STATUS     )   \
                            | PIN_PUPDR_FLOATING( GPIOB_LED_SDCARD     ) )

#define VAL_GPIOB_ODR       ( PIN_ODR_LOW( GPIOB_HMC5883L_DRDY  )          \
                            | PIN_ODR_LOW( GPIOB_NC_1           )          \
                            | PIN_ODR_LOW( GPIOB_NC_2           )          \
                            | PIN_ODR_LOW( GPIOB_JTAG_TDO       )          \
                            | PIN_ODR_LOW( GPIOB_JTAG_TRST      )          \
                            | PIN_ODR_LOW( GPIOB_IO_POWER_EN    )          \
                            | PIN_ODR_LOW( GPIOB_UART1_TX_CONN1 )          \
                            | PIN_ODR_LOW( GPIOB_UART1_RX_CONN1 )          \
                            | PIN_ODR_LOW( GPIOB_I2C1_SCL       )          \
                            | PIN_ODR_LOW( GPIOB_I2C1_SDA       )          \
                            | PIN_ODR_LOW( GPIOB_I2C2_SCL_CONN  )          \
                            | PIN_ODR_LOW( GPIOB_I2C2_SDA_CONN  )          \
                            | PIN_ODR_LOW( GPIOB_CAN2_RX_CONN   )          \
                            | PIN_ODR_LOW( GPIOB_CAN2_TX_CONN   )          \
                            | PIN_ODR_LOW( GPIOB_LED_STATUS     )          \
                            | PIN_ODR_LOW( GPIOB_LED_SDCARD     ) )

#define VAL_GPIOB_AFRL      ( PIN_AFIO_AF( GPIOB_HMC5883L_DRDY  ,  0)      \
                            | PIN_AFIO_AF( GPIOB_NC_1           ,  0)      \
                            | PIN_AFIO_AF( GPIOB_NC_2           ,  0)      \
                            | PIN_AFIO_AF( GPIOB_JTAG_TDO       ,  0)      \
                            | PIN_AFIO_AF( GPIOB_JTAG_TRST      ,  0)      \
                            | PIN_AFIO_AF( GPIOB_IO_POWER_EN    ,  0)      \
                            | PIN_AFIO_AF( GPIOB_UART1_TX_CONN1 ,  7)      \
                            | PIN_AFIO_AF( GPIOB_UART1_RX_CONN1 ,  7) )

#define VAL_GPIOB_AFRH      ( PIN_AFIO_AF( GPIOB_I2C1_SCL       ,  0)      \
                            | PIN_AFIO_AF( GPIOB_I2C1_SDA       ,  0)      \
                            | PIN_AFIO_AF( GPIOB_I2C2_SCL_CONN  ,  0)      \
                            | PIN_AFIO_AF( GPIOB_I2C2_SDA_CONN  ,  0)      \
                            | PIN_AFIO_AF( GPIOB_CAN2_RX_CONN   ,  0)      \
                            | PIN_AFIO_AF( GPIOB_CAN2_TX_CONN   ,  0)      \
                            | PIN_AFIO_AF( GPIOB_LED_STATUS     ,  0)      \
                            | PIN_AFIO_AF( GPIOB_LED_SDCARD     ,  0) )


// GPIO C

#define VAL_GPIOC_MODER     ( PIN_MODE_INPUT(     GPIOC_VBAT_MON_AIN    )       \
                            | PIN_MODE_INPUT(     GPIOC_SDCARD_DETECT   )       \
                            | PIN_MODE_INPUT(     GPIOC_MPU6000_INT     )       \
                            | PIN_MODE_INPUT(     GPIOC_MPU6000_FSYNC   )       \
                            | PIN_MODE_INPUT(     GPIOC_MPU6000_CS      )       \
                            | PIN_MODE_INPUT(     GPIOC_CAN_CONN_EN     )       \
                            | PIN_MODE_ALTERNATE( GPIOC_UART6_TX_CONN4  )       \
                            | PIN_MODE_ALTERNATE( GPIOC_UART6_RX_CONN4  )       \
                            | PIN_MODE_INPUT(     GPIOC_SDIO_D0         )       \
                            | PIN_MODE_INPUT(     GPIOC_SDIO_D1         )       \
                            | PIN_MODE_INPUT(     GPIOC_SDIO_D2         )       \
                            | PIN_MODE_INPUT(     GPIOC_SDIO_D3         )       \
                            | PIN_MODE_INPUT(     GPIOC_SDIO_CK         )       \
                            | PIN_MODE_INPUT(     GPIOC_SDCARD_POWER_EN )       \
                            | PIN_MODE_INPUT(     GPIOC_VCC_A_POWER_EN  )       \
                            | PIN_MODE_INPUT(     GPIOC_H3LIS331DL_INT  ) )


#define VAL_GPIOC_OTYPER    ( PIN_OTYPE_PUSHPULL( GPIOC_VBAT_MON_AIN    )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_SDCARD_DETECT   )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_MPU6000_INT     )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_MPU6000_FSYNC   )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_MPU6000_CS      )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_CAN_CONN_EN     )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_UART6_TX_CONN4  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_UART6_RX_CONN4  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_SDIO_D0         )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_SDIO_D1         )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_SDIO_D2         )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_SDIO_D3         )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_SDIO_CK         )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_SDCARD_POWER_EN )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_VCC_A_POWER_EN  )   \
                            | PIN_OTYPE_PUSHPULL( GPIOC_H3LIS331DL_INT  ) )

#define VAL_GPIOC_OSPEEDR   ( PIN_OSPEED_100M( GPIOC_VBAT_MON_AIN    )      \
                            | PIN_OSPEED_100M( GPIOC_SDCARD_DETECT   )      \
                            | PIN_OSPEED_100M( GPIOC_MPU6000_INT     )      \
                            | PIN_OSPEED_100M( GPIOC_MPU6000_FSYNC   )      \
                            | PIN_OSPEED_100M( GPIOC_MPU6000_CS      )      \
                            | PIN_OSPEED_100M( GPIOC_CAN_CONN_EN     )      \
                            | PIN_OSPEED_100M( GPIOC_UART6_TX_CONN4  )      \
                            | PIN_OSPEED_100M( GPIOC_UART6_RX_CONN4  )      \
                            | PIN_OSPEED_100M( GPIOC_SDIO_D0         )      \
                            | PIN_OSPEED_100M( GPIOC_SDIO_D1         )      \
                            | PIN_OSPEED_100M( GPIOC_SDIO_D2         )      \
                            | PIN_OSPEED_100M( GPIOC_SDIO_D3         )      \
                            | PIN_OSPEED_100M( GPIOC_SDIO_CK         )      \
                            | PIN_OSPEED_100M( GPIOC_SDCARD_POWER_EN )      \
                            | PIN_OSPEED_100M( GPIOC_VCC_A_POWER_EN  )      \
                            | PIN_OSPEED_100M( GPIOC_H3LIS331DL_INT  ) )

#define VAL_GPIOC_PUPDR     ( PIN_PUPDR_FLOATING( GPIOC_VBAT_MON_AIN    )   \
                            | PIN_PUPDR_FLOATING( GPIOC_SDCARD_DETECT   )   \
                            | PIN_PUPDR_FLOATING( GPIOC_MPU6000_INT     )   \
                            | PIN_PUPDR_FLOATING( GPIOC_MPU6000_FSYNC   )   \
                            | PIN_PUPDR_FLOATING( GPIOC_MPU6000_CS      )   \
                            | PIN_PUPDR_FLOATING( GPIOC_CAN_CONN_EN     )   \
                            | PIN_PUPDR_FLOATING( GPIOC_UART6_TX_CONN4  )   \
                            | PIN_PUPDR_PULLUP(   GPIOC_UART6_RX_CONN4  )   \
                            | PIN_PUPDR_FLOATING( GPIOC_SDIO_D0         )   \
                            | PIN_PUPDR_FLOATING( GPIOC_SDIO_D1         )   \
                            | PIN_PUPDR_FLOATING( GPIOC_SDIO_D2         )   \
                            | PIN_PUPDR_FLOATING( GPIOC_SDIO_D3         )   \
                            | PIN_PUPDR_FLOATING( GPIOC_SDIO_CK         )   \
                            | PIN_PUPDR_FLOATING( GPIOC_SDCARD_POWER_EN )   \
                            | PIN_PUPDR_FLOATING( GPIOC_VCC_A_POWER_EN  )   \
                            | PIN_PUPDR_FLOATING( GPIOC_H3LIS331DL_INT  ) )

#define VAL_GPIOC_ODR       ( PIN_ODR_LOW( GPIOC_VBAT_MON_AIN    )          \
                            | PIN_ODR_LOW( GPIOC_SDCARD_DETECT   )          \
                            | PIN_ODR_LOW( GPIOC_MPU6000_INT     )          \
                            | PIN_ODR_LOW( GPIOC_MPU6000_FSYNC   )          \
                            | PIN_ODR_LOW( GPIOC_MPU6000_CS      )          \
                            | PIN_ODR_LOW( GPIOC_CAN_CONN_EN     )          \
                            | PIN_ODR_LOW( GPIOC_UART6_TX_CONN4  )          \
                            | PIN_ODR_LOW( GPIOC_UART6_RX_CONN4  )          \
                            | PIN_ODR_LOW( GPIOC_SDIO_D0         )          \
                            | PIN_ODR_LOW( GPIOC_SDIO_D1         )          \
                            | PIN_ODR_LOW( GPIOC_SDIO_D2         )          \
                            | PIN_ODR_LOW( GPIOC_SDIO_D3         )          \
                            | PIN_ODR_LOW( GPIOC_SDIO_CK         )          \
                            | PIN_ODR_LOW( GPIOC_SDCARD_POWER_EN )          \
                            | PIN_ODR_LOW( GPIOC_VCC_A_POWER_EN  )          \
                            | PIN_ODR_LOW( GPIOC_H3LIS331DL_INT  ) )

#define VAL_GPIOC_AFRL      ( PIN_AFIO_AF( GPIOC_VBAT_MON_AIN    ,  0)      \
                            | PIN_AFIO_AF( GPIOC_SDCARD_DETECT   ,  0)      \
                            | PIN_AFIO_AF( GPIOC_MPU6000_INT     ,  0)      \
                            | PIN_AFIO_AF( GPIOC_MPU6000_FSYNC   ,  0)      \
                            | PIN_AFIO_AF( GPIOC_MPU6000_CS      ,  0)      \
                            | PIN_AFIO_AF( GPIOC_CAN_CONN_EN     ,  0)      \
                            | PIN_AFIO_AF( GPIOC_UART6_TX_CONN4  ,  8)      \
                            | PIN_AFIO_AF( GPIOC_UART6_RX_CONN4  ,  8) )

#define VAL_GPIOC_AFRH      ( PIN_AFIO_AF( GPIOC_SDIO_D0         ,  0)      \
                            | PIN_AFIO_AF( GPIOC_SDIO_D1         ,  0)      \
                            | PIN_AFIO_AF( GPIOC_SDIO_D2         ,  0)      \
                            | PIN_AFIO_AF( GPIOC_SDIO_D3         ,  0)      \
                            | PIN_AFIO_AF( GPIOC_SDIO_CK         ,  0)      \
                            | PIN_AFIO_AF( GPIOC_SDCARD_POWER_EN ,  0)      \
                            | PIN_AFIO_AF( GPIOC_VCC_A_POWER_EN  ,  0)      \
                            | PIN_AFIO_AF( GPIOC_H3LIS331DL_INT  ,  0) )


// GPIO D

#define VAL_GPIOD_MODER     ( PIN_MODE_INPUT( GPIOD_SDIO_CMD ) )

#define VAL_GPIOD_OTYPER    ( PIN_OTYPE_PUSHPULL( GPIOD_SDIO_CMD ) )

#define VAL_GPIOD_OSPEEDR   ( PIN_OSPEED_100M( GPIOD_SDIO_CMD ) )

#define VAL_GPIOD_PUPDR     ( PIN_PUPDR_FLOATING( GPIOD_SDIO_CMD ) )

#define VAL_GPIOD_ODR       ( PIN_ODR_LOW( GPIOD_SDIO_CMD ) )

#define VAL_GPIOD_AFRL      ( PIN_AFIO_AF( GPIOD_SDIO_CMD,  0) )

#define VAL_GPIOD_AFRH      ( 0 )


// GPIO E

#define VAL_GPIOE_MODER     ( 0 )

#define VAL_GPIOE_OTYPER    ( 0 )

#define VAL_GPIOE_OSPEEDR   ( 0 )

#define VAL_GPIOE_PUPDR     ( 0 )

#define VAL_GPIOE_ODR       ( 0 )

#define VAL_GPIOE_AFRL      ( 0 )

#define VAL_GPIOE_AFRH      ( 0 )


// GPIO F

#define VAL_GPIOF_MODER     ( 0 )

#define VAL_GPIOF_OTYPER    ( 0 )

#define VAL_GPIOF_OSPEEDR   ( 0 )

#define VAL_GPIOF_PUPDR     ( 0 )

#define VAL_GPIOF_ODR       ( 0 )

#define VAL_GPIOF_AFRL      ( 0 )

#define VAL_GPIOF_AFRH      ( 0 )


// GPIO G

#define VAL_GPIOG_MODER     ( 0 )

#define VAL_GPIOG_OTYPER    ( 0 )

#define VAL_GPIOG_OSPEEDR   ( 0 )

#define VAL_GPIOG_PUPDR     ( 0 )

#define VAL_GPIOG_ODR       ( 0 )

#define VAL_GPIOG_AFRL      ( 0 )

#define VAL_GPIOG_AFRH      ( 0 )


// GPIO H

#define VAL_GPIOH_MODER     ( PIN_MODE_INPUT( GPIOH_OSC_IN      )           \
                            | PIN_MODE_INPUT( GPIOH_OSC_OUT     ) )

#define VAL_GPIOH_OTYPER    ( PIN_OTYPE_PUSHPULL( GPIOH_OSC_IN      )       \
                            | PIN_OTYPE_PUSHPULL( GPIOH_OSC_OUT     ) )

#define VAL_GPIOH_OSPEEDR   ( PIN_OSPEED_100M( GPIOH_OSC_IN      )          \
                            | PIN_OSPEED_100M( GPIOH_OSC_OUT     ) )

#define VAL_GPIOH_PUPDR     ( PIN_PUPDR_FLOATING( GPIOH_OSC_IN      )       \
                            | PIN_PUPDR_FLOATING( GPIOH_OSC_OUT     ) )

#define VAL_GPIOH_ODR       ( PIN_ODR_LOW( GPIOH_OSC_IN      )              \
                            | PIN_ODR_LOW( GPIOH_OSC_OUT     ) )

#define VAL_GPIOH_AFRL      ( PIN_AFIO_AF( GPIOH_OSC_IN    , 0)             \
                            | PIN_AFIO_AF( GPIOH_OSC_OUT   , 0) )

#define VAL_GPIOH_AFRH      ( 0 )


// GPIO I

#define VAL_GPIOI_MODER     ( 0 )

#define VAL_GPIOI_OTYPER    ( 0 )

#define VAL_GPIOI_OSPEEDR   ( 0 )

#define VAL_GPIOI_PUPDR     ( 0 )

#define VAL_GPIOI_ODR       ( 0 )

#define VAL_GPIOI_AFRL      ( 0 )

#define VAL_GPIOI_AFRH      ( 0 )


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
