#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <string.h>
#include <arm-cortex-tools/mpu.h>
#include <arm-cortex-tools/fault.h>
#include "panic_handler.h"
#include "run_shell.h"
#include "log.h"
#include "actuators.hpp"
#include "thread_prio.h"
#include "parameter_storage.h"
#include "sumd_input.hpp"
#include "timestamp_stm32.h"
#include "ros_comm.hpp"
#include "control_loop.hpp"
#include "rate_controller_pid.hpp"
#include "output_mixer_linear.hpp"
#include "hott_tm.hpp"
#include "lis3mdl_publisher.hpp"
#include "icm20602_publisher.hpp"
#include "arm_led.h"
#include "usbcfg.h"

void dbg_enter_irq(void) {
    // palSetPad(GPIOA, GPIOA_PIN15_SPI_CONN_S0);
}

void dbg_leave_irq(void) {
    // palClearPad(GPIOA, GPIOA_PIN15_SPI_CONN_S0);
}

void dbg_enter_idle(void) {
    // palSetPad(GPIOB, GPIOB_PIN0_SPI_CONN_S1);
}

void dbg_leave_idle(void) {
    // palClearPad(GPIOB, GPIOB_PIN0_SPI_CONN_S1);
}


static THD_WORKING_AREA(blinking_thread_wa, 128);
static THD_FUNCTION(blinking_thread, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palClearPad(GPIOA, GPIOA_SWCLK_LED1);
        chThdSleepMilliseconds(100);
        palSetPad(GPIOA, GPIOA_SWCLK_LED1);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOA, GPIOA_SWCLK_LED1);
        chThdSleepMilliseconds(100);
        palSetPad(GPIOA, GPIOA_SWCLK_LED1);
        chThdSleepMilliseconds(700);
    }
}

// internal I2C
static const uint32_t i2c_presc = 8; // 72/(8+1) = 8Mhz, follow fast mode 400kHz example in ref manual
static const uint32_t i2c_scll = 0x9;
static const uint32_t i2c_sclh = 0x3;
static const uint32_t i2c_sdadel = 0x1;
static const uint32_t i2c_scldel = 0x3;
static const I2CConfig i2c_cfg = {
    .timingr = i2c_scll + (i2c_sclh<<8) + (i2c_sdadel<<16) + (i2c_scldel<<20) + (i2c_presc<<28),
    .cr1 = 0,
    .cr2 = 0
};

static void init_interfaces()
{
    SerialConfig uart_config = { .speed=SERIAL_DEFAULT_BITRATE, .cr1=0,
                                 .cr2=USART_CR2_STOP1_BITS, .cr3=USART_CR3_RTSE | USART_CR3_CTSE };
    // uart1 connected to receiver sumd
    uart_config.speed = 115200;
    sdStart(&SD1, &uart_config);
    // uart2 connected to nanopi
    uart_config.speed = 1500000;
    sdStart(&SD2, &uart_config);
    // uart3 connected to receiver telemetry
    uart_config.speed = 19200;
    sdStart(&SD3, &uart_config);
    // uart6 connected to debug/shell
    uart_config.speed = 115200;
    sdStart(&SD6, &uart_config);
    // uart4 on uart2 flow control pins, not used
    uart_config.speed = 115200;
    // sdStart(&SD4, &uart_config);

    i2cStart(&I2CD1, &i2c_cfg);
}

log_handler_t log_handler_stdout;
static void log_handler_stdout_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    streamWrite((BaseSequentialStream*)&SD6, (uint8_t*)msg, len);
}

int main(void) {
    halInit();
    chSysInit();

    panic_handler_init(SD6.usart);
    // mpu_init();
    // fault_init();

    init_interfaces();
#ifndef DEBUG
    palSetPadMode(GPIOA, GPIOA_SWCLK_LED1, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOA, GPIOA_SWDIO_LED2, PAL_MODE_OUTPUT_PUSHPULL);
#endif

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    /*
    * Activates the USB driver and then the USB bus pull-up on D+.
    * Note, a delay is inserted in order to not have to disconnect the cable
    * after a reset.
    */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    chThdSleepMilliseconds(5000);

    timestamp_stm32_init();
    parameter_init(&I2CD1, 0x50);

    log_init();
    log_handler_register(&log_handler_stdout, LOG_LVL_DEBUG, log_handler_stdout_cb);

    // ros_comm_init(&SD2);
    ros_comm_init((BaseSequentialStream*)&SDU1);

    log_info("=== boot ===");

    const char *panic_msg = get_panic_message();
    if (panic_msg) {
        log_error("Reboot after panic: %s", panic_msg);
    }
    chThdCreateStatic(blinking_thread_wa, sizeof(blinking_thread_wa), THD_PRIO_LED, blinking_thread, NULL);

    control_init();
    initialize_actuators(&parameters);

    static PIDRateController rate_ctrl;
    static LinearOutputMixer mixer;
    rate_ctrl.declare_parameters(&control_ns);
    mixer.declare_parameters(&control_ns);

    if (parameter_load_from_persistent_store()) {
        log_info("parameters loaded");
    }

    arm_led_task_start();

    run_shell((BaseSequentialStream*)&SD6);
    sumd_input_start((BaseSequentialStream*)&SD1);

    control_start(rate_ctrl, mixer);

    hott_tm_start((BaseSequentialStream*)&SD3);

    SPIConfig icm20602_spi_config={
        .circular = false,
        .end_cb = NULL,
        .ssport = GPIOC,
        .sspad = GPIOC_PIN15_GYRO2_CS,
        .cr1 =  SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA, // ~6.7MHz
        .cr2 = 0,
    };
    Eigen::Matrix3f R_icm20602_to_board;
    R_icm20602_to_board << 0, -1, 0,
                          1, 0, 0,
                          0, 0, 1;
    icm20602_publisher_start(&SPID1, &icm20602_spi_config, R_icm20602_to_board, PAL_LINE(GPIOC, GPIOC_PIN3_GYRO2_INT));

    Eigen::Matrix3f R_lis3mdl_to_board; // todo
    R_lis3mdl_to_board << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 1;
    lis3mdl_publisher_start(&I2CD1, 0x1E, R_lis3mdl_to_board);


    while (true) {
        chThdSleepMilliseconds(1000);
    }
}
