#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include "serial-datagram/serial_datagram.h"
#include "serial-datagram/serial_datagram_buffer_writer.h"
#include "net/net.h"
#include <string.h>

#include "usbcfg.h"

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


#define NET_MTU 1000

net_node_t net_node;

char port0_rx_buf[NET_MTU];
serial_datagram_rcv_handler_t port0_rx;
int port_0_if_idx = 0;
char port1_rx_buf[NET_MTU];
serial_datagram_rcv_handler_t port1_rx;
int port_1_if_idx = 1;

void serial_datagram_rx_cb(const void *dtgrm, size_t len, void *arg)
{
    int *if_idx = (int*)arg;
    // chprintf((BaseSequentialStream *)&SD2, "[debug] received if %d,  %lu: %s\n", *if_idx, len, (char*)dtgrm);
    net_handle_incoming_frame(&net_node, dtgrm, len, *if_idx, 0);
}

void serial_if_tx(void *arg, const char *frame, size_t len, uint8_t prio, uint8_t dest)
{
    (void)prio;
    (void)dest;
    if (arg == NULL) {
        return;
    }

    // chprintf((BaseSequentialStream *)&SD2, "[debug] sending to %x, if %x\n", dest, arg);

    char send_buf[2*NET_MTU];
    size_t send_len = serial_datagram_buffer_wrap((uint8_t*)frame, len, (uint8_t*)send_buf, sizeof(send_buf));
    streamWrite((BaseSequentialStream*)arg, (uint8_t*)send_buf, send_len);
}

#define ROUTING_TABLE_SIZE 10
struct net_route_tab_entry_s routing_table[ROUTING_TABLE_SIZE] = {
    {.link_layer_via_interface_idx = -1}
};

struct net_if_s if_list[2] = {
    {.send_fn = serial_if_tx, .arg = NULL},
    // {.send_fn = serial_if_tx, .arg = NULL},
    {.send_fn = NULL}
};


void protocol0(net_node_t *node, const char *pkt, size_t len, uint8_t src_addr, uint8_t prio, uint8_t interface_idx)
{
    (void)node;
    (void)interface_idx;
    // chprintf((BaseSequentialStream *)&SD2, "ping from %d, prio %d, via if %d\n", src_addr, prio, interface_idx);
    // echo
    static char reply[NET_MTU];
    memcpy(&reply[NET_HEADER_LEN], pkt, len);
    const int protocol = 0;
    net_write_header_and_send_frame(&net_node, protocol, reply, len+NET_HEADER_LEN, src_addr, prio);
}


void protocol1(net_node_t *node, const char *pkt, size_t len, uint8_t src_addr, uint8_t prio, uint8_t interface_idx)
{
    (void)node;
    (void)pkt;
    (void)len;
    (void)src_addr;
    (void)prio;
    (void)interface_idx;
    // chprintf((BaseSequentialStream *)&SD2, "protocol 1 from %d, prio %d, via if %d\n", src_addr, prio, interface_idx);
    unsigned i;
    for (i = 0; i < len; i++) {
        chprintf((BaseSequentialStream *)&SD2, "  %c %x\n", pkt[i], pkt[i]);
    }
}

const struct net_protocol_table_entry_s protocol_table[] = {
    {0, protocol0},
    {1, protocol1},
    {.protocol_nbr = -1}
};


void net_init(uint8_t addr)
{
    net_node_init(&net_node, addr, routing_table,
                  sizeof(routing_table)/sizeof(struct net_route_tab_entry_s),
                  if_list,
                  protocol_table);
}

static THD_WORKING_AREA(net_rx0_thd_wa, (2000+2*NET_MTU));
static THD_FUNCTION(net_rx0_thd, arg) {
    if_list[0].arg =  arg;
    serial_datagram_rcv_handler_init(&port0_rx, port0_rx_buf, sizeof(port0_rx_buf), serial_datagram_rx_cb, &port_0_if_idx);
    while (1) {
        uint8_t buf;
        size_t len = streamRead((BaseSequentialStream*)arg, &buf, 1);
        // chprintf((BaseSequentialStream *)&SD2, "rx %c\n", buf);
        serial_datagram_receive(&port0_rx, &buf, len);
        if (len == 0) {
            chThdSleepMilliseconds(10); // queue is probably reset, avoid busy loop
        }
    }
}
// static THD_WORKING_AREA(net_rx1_thd_wa, (2000+2*NET_MTU));
// static THD_FUNCTION(net_rx1_thd, arg) {
//     if_list[1].arg = arg;
//     serial_datagram_rcv_handler_init(&port1_rx, port1_rx_buf, sizeof(port1_rx_buf), serial_datagram_rx_cb, &port_1_if_idx);
//     while (1) {
//         uint8_t buf;
//         size_t len = streamRead((BaseSequentialStream*)arg, &buf, 1);
//         serial_datagram_receive(&port1_rx, &buf, len);
//         if (len == 0) {
//             chThdSleepMilliseconds(10); // queue is probably reset, avoid busy loop
//         }
//     }
// }



int main(void) {
    halInit();
    chSysInit();

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO - 1, Thread1, NULL);

    uint8_t my_addr;//, other_addr;
    if (true) {
        my_addr = 0x42;
        // other_addr = 0x43;
    } else {
        my_addr = 0x43;
        // other_addr = 0x42;
    }

    SerialConfig uart_config = { SERIAL_DEFAULT_BITRATE, 0,
                                 USART_CR2_STOP1_BITS, 0 };
    uart_config.speed = 115200;
    uart_config.speed = 1500000;

    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7)); // TX
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7) + PAL_STM32_PUPDR_PULLUP); // RX
    sdStart(&SD1, &uart_config);
    sdStart(&SD2, NULL); // connected to stlink

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    // usbDisconnectBus(serusbcfg.usbp);
    // chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    // usbConnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(100);

    while (true) {
        uint8_t buf;
        size_t len = streamRead((BaseSequentialStream*)&SD1, &buf, 1);
        if (len == 0) {
            chThdSleepMilliseconds(10); // queue is probably reset, avoid busy loop
        } else {
            buf = ~buf;
            streamWrite((BaseSequentialStream*)&SD1, &buf, 1);
        }
    }


    net_init(my_addr);
    // interface 0, USB
    chThdCreateStatic(net_rx0_thd_wa, sizeof(net_rx0_thd_wa),
                      NORMALPRIO - 1, net_rx0_thd, &SDU1);
    // // interface 1 UART connection to other node
    // chThdCreateStatic(net_rx1_thd_wa, sizeof(net_rx1_thd_wa),
    //                   NORMALPRIO - 1, net_rx1_thd, &SD2);
    // net_route_add(&net_node, other_addr, 0xff, 1, 0);

    int i = 0;
    while (true) {
        chprintf((BaseSequentialStream *)&SD2, "node %x, counting %d\n", my_addr, i++);
        chThdSleepMilliseconds(500);
    }
}
