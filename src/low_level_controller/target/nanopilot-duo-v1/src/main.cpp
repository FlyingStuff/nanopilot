#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <string.h>
#include <arm-cortex-tools/mpu.h>
#include <arm-cortex-tools/fault.h>
#include "panic_handler.h"
#include "run_shell.h"
#include "log.h"
#include "actuators_driver.hpp"
#include "thread_prio.h"
#include "parameter_storage.h"
#include "sumd_input.hpp"
#include "timestamp_stm32.h"
#include "ros_comm.hpp"
#include "control_loop.hpp"
#include "attitude_controller.hpp"
#include "hott_tm.hpp"
#include "lis3mdl_publisher.hpp"
#include "icm20602_publisher.hpp"
#include "arm_led.h"
#include "adc.hpp"

void dbg_enter_irq(void) {
    palSetPad(GPIOA, GPIOA_PIN15_SPI_CONN_S0);
}

void dbg_leave_irq(void) {
    palClearPad(GPIOA, GPIOA_PIN15_SPI_CONN_S0);
}

void dbg_enter_idle(void) {
    palSetPad(GPIOB, GPIOB_PIN0_SPI_CONN_S1);
}

void dbg_leave_idle(void) {
    palClearPad(GPIOB, GPIOB_PIN0_SPI_CONN_S1);
}


static THD_WORKING_AREA(blinking_thread_wa, 128);
static THD_FUNCTION(blinking_thread, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palSetPad(GPIOB, GPIOB_PIN12_HEARTBEAT_LED);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOB, GPIOB_PIN12_HEARTBEAT_LED);
        chThdSleepMilliseconds(100);
        palSetPad(GPIOB, GPIOB_PIN12_HEARTBEAT_LED);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOB, GPIOB_PIN12_HEARTBEAT_LED);
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
    // uart1 connected to nanopi
    uart_config.speed = 1500000;
    sdStart(&SD1, &uart_config);
    // uart3 connected to receiver telemetry
    uart_config.speed = 19200;
    sdStart(&SD3, &uart_config);
    // uart5 connected to receiver sumd
    uart_config.speed = 115200;
    sdStart(&SD5, &uart_config);
    // uart6 connected to debug/shell
    uart_config.speed = 115200;
    sdStart(&SD6, &uart_config);
    // uart4 on SPI connector
    uart_config.speed = 115200;
    sdStart(&SD4, &uart_config);

    i2cStart(&I2CD1, &i2c_cfg);
    i2cStart(&I2CD3, &i2c_cfg);
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

    timestamp_stm32_init();
    parameter_init(&I2CD3, 0x50);

    log_init();
    log_handler_register(&log_handler_stdout, LOG_LVL_DEBUG, log_handler_stdout_cb);

    ros_comm_init((BaseSequentialStream*)&SD1);

    log_info("=== boot ===");

    const char *panic_msg = get_panic_message();
    if (panic_msg) {
        log_error("Reboot after panic: %s", panic_msg);
    }
    chThdCreateStatic(blinking_thread_wa, sizeof(blinking_thread_wa), THD_PRIO_LED, blinking_thread, NULL);

    control_init();
    actuators_init(&parameters);

    run_adc();

    icm20602_parameter_declare(&parameters);
    static AttitudeController att_controller;
    att_controller.declare_parameters(&control_ns);

    if (parameter_load_from_persistent_store()) {
        log_info("parameters loaded");
    }

    arm_led_task_start();

    run_shell((BaseSequentialStream*)&SD6);
    sumd_input_start((BaseSequentialStream*)&SD5);

    control_start(att_controller);

    hott_tm_start((BaseSequentialStream*)&SD3);

    SPIConfig icm20602_spi_config={
        .circular = false,
        .end_cb = NULL,
        .ssport = GPIOC,
        .sspad = GPIOC_PIN15_SPI_ICM20602_CS,
        .cr1 =  SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA, // ~6.7MHz
        .cr2 = 0,
    };
    Eigen::Matrix3f R_icm20602_to_board;
    R_icm20602_to_board << 0, -1, 0,
                          1, 0, 0,
                          0, 0, 1;
    icm20602_publisher_start(&SPID3, &icm20602_spi_config, R_icm20602_to_board, PAL_LINE(GPIOC, GPIOC_PIN14_ICM20602_INT));

    Eigen::Matrix3f R_lis3mdl_to_board; // todo
    R_lis3mdl_to_board << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 1;
    lis3mdl_publisher_start(&I2CD3, 0x1E, R_lis3mdl_to_board);


    while (true) {
        chThdSleepMilliseconds(1000);
    }
}
