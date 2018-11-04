#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <string.h>
#include <math.h>
#include <ros_interface/comm.h>
#include <ros_interface/msg.h>

#include "usbcfg.h"


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


static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palSetLine(LINE_LED_GREEN);
        chThdSleepMilliseconds(200);
        palClearLine(LINE_LED_GREEN);
        chThdSleepMilliseconds(200);
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
    while (1) {
        comm_receive(&comm_if);
    }
}


int main(void) {
    halInit();
    chSysInit();

    palSetLineMode(LINE_ARD_D4, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(LINE_ARD_D3, PAL_MODE_OUTPUT_PUSHPULL);


    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO - 1, Thread1, NULL);

    SerialConfig uart_config = { .speed=SERIAL_DEFAULT_BITRATE, .cr1=0,
                                 .cr2=USART_CR2_STOP1_BITS, .cr3=USART_CR3_RTSE | USART_CR3_CTSE };
    // uart_config.speed = 9600;
    // uart_config.speed = 115200;
    uart_config.speed = 1500000;

    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7)); // TX
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // RX
    sdStart(&SD1, &uart_config);
    // uart2
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); // TX
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // RX
    palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // CTS
    palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(7)); // RTS
    sdStart(&SD2, &uart_config);

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    // usbDisconnectBus(serusbcfg.usbp);
    // chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    // usbConnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);


    comm_init(&comm_if, (BaseSequentialStream*)&SD2, comm_rcv_cb);

    chThdCreateStatic(comm_rx_thread_wa, sizeof(comm_rx_thread_wa),
                      NORMALPRIO, comm_rx_thread, NULL);


    int i=0;
    while (true) {
        // comm_send(&comm_if, RosInterfaceCommMsgID::HEARTBEAT, NULL, 0);
        uint64_t timestamp = i;
        comm_send(&comm_if, RosInterfaceCommMsgID::TIME, &timestamp, sizeof(timestamp));

        static char buf[1000];
        auto serializer = nop::Serializer<nop::BufferWriter>(buf, sizeof(buf));
        serializer.Write(SimpleType{static_cast<uint32_t>(i), sinf(i)});
        comm_send(&comm_if, RosInterfaceCommMsgID::TEST, buf, serializer.writer().size());

        chThdSleepMilliseconds(1);
        i++;
    }
}
