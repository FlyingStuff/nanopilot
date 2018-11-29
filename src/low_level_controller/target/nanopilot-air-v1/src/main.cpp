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
#include "hott_tm.hpp"
#include "lsm6dsm_publisher.hpp"
#include "arm_led.h"

void dbg_enter_irq(void) {
    palSetPad(GPIOE, GPIOE_PIN11_TP4);
}

void dbg_leave_irq(void) {
    palClearPad(GPIOE, GPIOE_PIN11_TP4);
}

void dbg_enter_idle(void) {
    palSetPad(GPIOE, GPIOE_PIN12_TP5);
}

void dbg_leave_idle(void) {
    palClearPad(GPIOE, GPIOE_PIN12_TP5);
}


static THD_WORKING_AREA(blinking_thread_wa, 128);
static THD_FUNCTION(blinking_thread, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palSetPad(GPIOE, GPIOE_PIN8_HEARTBEAT_LED);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOE, GPIOE_PIN8_HEARTBEAT_LED);
        chThdSleepMilliseconds(100);
        palSetPad(GPIOE, GPIOE_PIN8_HEARTBEAT_LED);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOE, GPIOE_PIN8_HEARTBEAT_LED);
        chThdSleepMilliseconds(700);
    }
}


static void init()
{
    SerialConfig uart_config = { .speed=SERIAL_DEFAULT_BITRATE, .cr1=0,
                                 .cr2=USART_CR2_STOP1_BITS, .cr3=USART_CR3_RTSE | USART_CR3_CTSE };
    // uart1 debug/shell
    uart_config.speed = 115200;
    sdStart(&SD1, &uart_config);
    // uart2 connected to nanopi
    uart_config.speed = 1500000;
    sdStart(&SD2, &uart_config);
    // uart5 connected to receiver sumd
    uart_config.speed = 115200;
    sdStart(&SD5, &uart_config);
    // uart3 connected to receiver telemetry
    uart_config.speed = 19200;
    sdStart(&SD3, &uart_config);
    // uart4 connected to GPS
    uart_config.speed = 9600;
    sdStart(&SD4, &uart_config);
}

log_handler_t log_handler_stdout;
static void log_handler_stdout_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    streamWrite((BaseSequentialStream*)&SD1, (uint8_t*)msg, len);
}


int main(void) {
    halInit();
    chSysInit();

    panic_handler_init(SD1.usart);
    mpu_init();
    fault_init();

    init();

    timestamp_stm32_init();
    parameter_init();

    log_init();
    log_handler_register(&log_handler_stdout, LOG_LVL_DEBUG, log_handler_stdout_cb);

    ros_comm_init(&SD2);

    log_info("=== boot ===");

    const char *panic_msg = get_panic_message();
    if (panic_msg) {
        log_error("Reboot after panic: %s", panic_msg);
    }
    chThdCreateStatic(blinking_thread_wa, sizeof(blinking_thread_wa), THD_PRIO_LED, blinking_thread, NULL);

    control_init();
    initialize_actuators(&parameters);

    // read_parameters_from_eeprom();

    arm_led_task_start();
    run_shell((BaseSequentialStream*)&SD1);
    sumd_input_start((BaseSequentialStream*)&SD5);
    control_start();
    hott_tm_start((BaseSequentialStream*)&SD3);

    SPIConfig lsm6dsm_spi_config={
        .end_cb = NULL,
        .ssport = GPIOD,
        .sspad = GPIOD_PIN10_SENS_LSM_CS,
        .cr1 =  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
        .cr2 = 0,
    };
    lsm6dsm_publisher_start(&SPID2, &lsm6dsm_spi_config);

    while (true) {
        chThdSleepMilliseconds(1000);
    }
}
