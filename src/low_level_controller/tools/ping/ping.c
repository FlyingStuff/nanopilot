#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <pthread.h>
#include <sys/mman.h>

#if defined(__linux__)
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif

#include "../../lib/comm/serial-datagram/serial_datagram.h"
#include "../../lib/comm/serial-datagram/serial_datagram_buffer_writer.h"
#include "../../lib/mcucom/net/net.h"

int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf("error %d from tcgetattr", errno);
            return -1;
    }
    // cfmakeraw(&tty);
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
            printf("error %d from tcsetattr", errno);
            return -1;
    }
#if defined(__linux__)
    printf("set linux low latency mode\n");
    struct serial_struct kernel_serial_settings;
    if (ioctl(fd, TIOCGSERIAL, &kernel_serial_settings) != 0) {
        printf("error %d from ioctl", errno);
        return -1;
    }
    kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(fd, TIOCSSERIAL, &kernel_serial_settings)) {
        printf("error %d from ioctl", errno);
        return -1;
    }
#endif
    return 0;
}


#include <sys/time.h>
#include "statistics.h"

statistics_t time_stats;
struct timeval  tv1, tv2;


#define NET_MTU 1000

net_node_t net_node;


void send_ping(int addr)
{
    char buf[NET_MTU];
    const char *msg = " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789";
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789"
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789"
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789"
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789"
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789"
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789"
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789"
                      // " 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789";
    memcpy(&buf[NET_HEADER_LEN], msg, strlen(msg));
    int protocol = 0;
    int prio = 0;
    gettimeofday(&tv1, NULL);
    if (!net_write_header_and_send_frame(&net_node, protocol, buf, strlen(msg)+NET_HEADER_LEN, addr, prio)) {
        printf("could not route ping packet\n");
    }


    if (statistics_get_nb_samples(&time_stats) >= 1000) {
        printf("_____________\n");

        printf("%d samples\n", statistics_get_nb_samples(&time_stats));
        printf("mean %f ms\n", statistics_get_mean(&time_stats));
        printf("stddev %f ms\n", statistics_get_stddev(&time_stats));
        printf("min %f ms\n", statistics_get_min(&time_stats));
        printf("max %f ms\n", statistics_get_max(&time_stats));
        statistics_reset(&time_stats);
    }

}


void protocol0(net_node_t *node, const char *pkt, size_t len, uint8_t src_addr, uint8_t prio, uint8_t interface_idx)
{
    (void)node;
    (void)interface_idx;
    gettimeofday(&tv2, NULL);
    float ms = (double) (tv2.tv_usec - tv1.tv_usec) / 1000 +
         (double) (tv2.tv_sec - tv1.tv_sec) * 1000;
        // printf ("Total time = %f ms\n", ms);
        statistics_add_sample(&time_stats, ms);

    send_ping(src_addr); // start next ping
    // printf("x\n");
}

void write_to_fd(void *arg, const char *frame, size_t len, uint8_t prio, uint8_t dest)
{
    (void)prio;
    (void)dest;
    int fd = *(int*)arg;
    char send_buf[2*NET_MTU];
    size_t send_len = serial_datagram_buffer_wrap((uint8_t*)frame, len, (uint8_t*)send_buf, sizeof(send_buf));
    size_t bytes_written = 0;
    // printf("write started %zu\n", send_len);
    while (bytes_written < send_len) {
        int ret = write(fd, &send_buf[bytes_written], send_len - bytes_written);
        if (ret == -1) {
            printf("write failed %d", errno);
            return;
        }
        bytes_written += ret;
    }
    // printf("write successful\n");
}


#define ROUTING_TABLE_SIZE 10
struct net_route_tab_entry_s routing_table[ROUTING_TABLE_SIZE] = {
    {.dest_addr=0, .dest_mask=0, .link_layer_via_interface_idx=0, .link_layer_via_addr=0}, // default route
    {.link_layer_via_interface_idx = -1}
};

const struct net_protocol_table_entry_s protocol_table[] = {
    {0, protocol0},
    {.protocol_nbr = -1}
};

void serial_datagram_rx_cb(const void *dtgrm, size_t len, void *arg)
{
    int *if_idx = (int*)arg;
    // printf("frame received\n");
    net_handle_incoming_frame(&net_node, dtgrm, len, *if_idx, 0);
}

int main(const int argc, const char **argv)
{
    if (argc != 2) {
        printf("usage: %s /dev/serialport\n", argv[0]);
        return -1;
    }
    const char *portname = argv[1];
    int fd = open (portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }
    printf("open ok\n");
    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    printf("set interface ok\n");
    // fcntl(fd, F_SETFL, 0); // blocking
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);



    // Lock memory to ensure no swapping is done.
    if(mlockall(MCL_FUTURE|MCL_CURRENT)){
            fprintf(stderr,"WARNING: Failed to lock memory\n");
    }
    // Set our thread to real time priority
    struct sched_param sp;
    sp.sched_priority = 30;
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)){
            fprintf(stderr,"WARNING: Failed to set thread to real-time priority\n");
    }


    struct net_if_s if_list[2] = {
        {.send_fn = write_to_fd, .arg = &fd},
        {.send_fn = NULL}
    };

    statistics_reset(&time_stats);

    int addr = 1;
    net_node_init(&net_node, addr, routing_table,
                  sizeof(routing_table)/sizeof(struct net_route_tab_entry_s),
                  if_list,
                  protocol_table);

    serial_datagram_rcv_handler_t rx;
    char rx_buf[NET_MTU];
    int rx_if_idx = 0;
    serial_datagram_rcv_handler_init(&rx, rx_buf, sizeof(rx_buf), serial_datagram_rx_cb, &rx_if_idx);

    printf("sending ping\n");
    // send_ping(0x42);
    send_ping(0x42);

    char buf[NET_MTU];
    while (1) {
        int ret = read(fd, buf, sizeof(buf));
        if (ret == -1) {
            printf("read failed %d", errno);
        } else {
            // printf("read %d bytes %s\n", ret, buf);
            serial_datagram_receive(&rx, &buf, ret);
        }
    }
    close(fd);
    return 0;
}


