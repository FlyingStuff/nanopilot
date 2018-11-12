#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <string.h>
#include <arm-cortex-tools/mpu.h>
#include <arm-cortex-tools/fault.h>
#include "panic_handler.h"
#include "run_shell.h"
#include "usbcfg.h"
#include "log.h"
#include "actuators.hpp"
#include "thread_prio.h"
#include "parameter_storage.h"
#include "sumd_input.hpp"
#include "timestamp_stm32.h"
#include "ros_comm.hpp"
#include "control_loop.hpp"


void dbg_enter_irq(void) {
    palSetLine(LINE_ARD_D4);
}

void dbg_leave_irq(void) {
    palClearLine(LINE_ARD_D4);
}

void dbg_enter_idle(void) {
    palSetLine(LINE_ARD_D3);
}

void dbg_leave_idle(void) {
    palClearLine(LINE_ARD_D3);
}


static THD_WORKING_AREA(blinking_thread_wa, 128);
static THD_FUNCTION(blinking_thread, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palSetLine(LINE_LED_GREEN);
        chThdSleepMilliseconds(100);
        palClearLine(LINE_LED_GREEN);
        chThdSleepMilliseconds(100);
        palSetLine(LINE_LED_GREEN);
        chThdSleepMilliseconds(100);
        palClearLine(LINE_LED_GREEN);
        chThdSleepMilliseconds(700);
    }
}


static void init()
{
    palSetLineMode(LINE_ARD_D4, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(LINE_ARD_D3, PAL_MODE_OUTPUT_PUSHPULL);

    SerialConfig uart_config = { .speed=SERIAL_DEFAULT_BITRATE, .cr1=0,
                                 .cr2=USART_CR2_STOP1_BITS, .cr3=USART_CR3_RTSE | USART_CR3_CTSE };

    // uart1 debug/shell
    uart_config.speed = 115200;
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7)); // TX
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // RX
    sdStart(&SD1, &uart_config);
    // uart2 connected to nanopi
    uart_config.speed = 1500000;
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); // TX
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // RX
    palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // CTS
    palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(7)); // RTS
    sdStart(&SD2, &uart_config);
    // uart3 connected to receiver sumd
    uart_config.speed = 115200;
    // palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7)); // TX
    palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // RX
    sdStart(&SD3, &uart_config);

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    // usbDisconnectBus(serusbcfg.usbp);
    // chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    // usbConnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);
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

    run_shell((BaseSequentialStream*)&SD1);
    sumd_input_start((BaseSequentialStream*)&SD3);
    control_start();

    while (true) {
        chThdSleepMilliseconds(1000);
    }
}
