#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <string.h>
#include <math.h>
#include <arm-cortex-tools/mpu.h>
#include <arm-cortex-tools/fault.h>
#include <ros_interface/comm.h>
#include <ros_interface/msg.h>
#include "panic_handler.h"
#include "run_shell.h"
#include "usbcfg.h"
#include "log.h"
#include "actuators.hpp"
#include "thread_prio.h"

#include "sumd_input.h"
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

static comm_interface_t comm_if;


void comm_rcv_cb(comm_msg_id_t msg_id, const uint8_t *msg, size_t len)
{
    switch (static_cast<RosInterfaceCommMsgID>(msg_id)) {
    case RosInterfaceCommMsgID::PING:
        comm_send(&comm_if, RosInterfaceCommMsgID::PONG, msg, len);
        break;
    default:
        break;
    }
}

static THD_WORKING_AREA(comm_rx_thread_wa, (2000));
static THD_FUNCTION(comm_rx_thread, arg) {
    (void)arg;
    chRegSetThreadName("comm_rx");
    while (1) {
        comm_receive(&comm_if);
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

    initialize_actuators();
}

static log_handler_t log_handler_stdout;
static void log_handler_stdout_cb(log_level_t lvl, const char *msg, size_t len)
{
    (void)lvl;
    streamWrite((BaseSequentialStream*)&SD1, (uint8_t*)msg, len);
}

// static log_handler_t log_handler_comm;
// static void log_handler_comm_cb(log_level_t lvl, const char *msg, size_t len)
// {
//     (void)lvl;
//     comm_send(&comm_if, RosInterfaceCommMsgID::DEBUG_, msg, len);
// }

int main(void) {
    halInit();
    chSysInit();

    panic_handler_init(SD1.usart);
    mpu_init();
    fault_init();

    init();

    comm_init(&comm_if, (BaseSequentialStream*)&SD2, comm_rcv_cb);

    log_init();
    log_handler_register(&log_handler_stdout, LOG_LVL_DEBUG, log_handler_stdout_cb);
    // log_handler_register(&log_handler_comm, LOG_LVL_DEBUG, log_handler_comm_cb);
    log_info("=== boot ===");
    const char *panic_msg = get_panic_message();
    if (panic_msg) {
        log_warning("Reboot after panic: %s", panic_msg);
    }

    chThdCreateStatic(blinking_thread_wa, sizeof(blinking_thread_wa), THD_PRIO_LED, blinking_thread, NULL);

    chThdCreateStatic(comm_rx_thread_wa, sizeof(comm_rx_thread_wa),
                      NORMALPRIO, comm_rx_thread, NULL);

    run_shell((BaseSequentialStream*)&SD1);

    sumd_input_start((BaseSequentialStream*)&SD3);

    auto sub_rc = msgbus::subscribe(rc_input);
    // int i=0;
    while (true) {
        sub_rc.wait_for_update();
        auto rc = sub_rc.get_value();
        log_debug("rc in %f %f %f", rc.channel[0], rc.channel[1], rc.channel[2]);

        actuators_set_output({1500 + 500*rc.channel[0], 1500, 1500, 1500});

        // comm_send(&comm_if, RosInterfaceCommMsgID::HEARTBEAT, NULL, 0);
        // uint64_t timestamp = i;
        // comm_send(&comm_if, RosInterfaceCommMsgID::TIME, &timestamp, sizeof(timestamp));

        // static char buf[1000];
        // auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
        // serializer.Write(SimpleType{static_cast<uint32_t>(i), sinf(i)});
        // comm_send(&comm_if, RosInterfaceCommMsgID::TEST, buf, serializer.writer().size());

        // chThdSleepMilliseconds(1);
        // i++;
    }
}
