#include <string.h>
#include <assert.h>

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#if defined(__linux__)
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif

#else // ChibiOS

#include <ch.h>
#include <hal.h>

#endif

#include <serial-datagram/serial_datagram_buffer_writer.h>

#include "comm.h"


static void datagram_rcv_cb(const uint8_t *dtgrm, size_t len, comm_interface_t *interface)
{
    comm_msg_id_t msg_id = 0;
    static_assert(sizeof(comm_msg_id_t) == 2, "adjust msg id serialization");
    msg_id |= (comm_msg_id_t)dtgrm[0] <<  0u;
    msg_id |= (comm_msg_id_t)dtgrm[1] <<  8u;
    interface->rcv_cb(msg_id, &dtgrm[sizeof(comm_msg_id_t)], len-sizeof(comm_msg_id_t));
}


#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))

static int rate_to_constant(int baudrate) {
#define B(x) case x: return B##x
    switch(baudrate) {
        B(50);     B(75);     B(110);    B(134);    B(150);
        B(200);    B(300);    B(600);    B(1200);   B(1800);
        B(2400);   B(4800);   B(9600);   B(19200);  B(38400);
        B(57600);  B(115200); B(230400); B(460800); B(500000);
        B(576000); B(921600); B(1000000);B(1152000);B(1500000);
    default: return 0;
    }
#undef B
}

static int set_interface_attribs (int fd, int speed, int parity)
{
    printf("setup\n");

    int oldfl;
    oldfl = fcntl(fd, F_GETFL);
    if (oldfl == -1) {
        printf("error %d from fcntl, %s\n", errno, strerror(errno));
    }
    fcntl(fd, F_SETFL, oldfl & ~O_NONBLOCK); // switch to blocking mode


    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf("error %d from tcgetattr, %s\n", errno, strerror(errno));
        return -1;
    }
    cfmakeraw(&tty);
    cfsetospeed (&tty, rate_to_constant(speed));
    cfsetispeed (&tty, rate_to_constant(speed));

    tty.c_cflag |= CRTSCTS;
    // tty.c_cflag &= ~CRTSCTS;

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag |= IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read waits for zero bytes
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr, %s\n", errno, strerror(errno));
        return -1;
    }

#if defined(__linux__)
    printf("set linux low latency mode\n");
    struct serial_struct kernel_serial_settings;
    if (ioctl(fd, TIOCGSERIAL, &kernel_serial_settings) != 0) {
        printf("error %d from ioctl, %s\n", errno, strerror(errno));
        return -1;
    }
    kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
    if (rate_to_constant(speed) == 0) {
        printf("custom divisor: baud base %d, divisor %d\n", kernel_serial_settings.baud_base, kernel_serial_settings.custom_divisor);
        kernel_serial_settings.flags &= ~ASYNC_SPD_MASK;
        kernel_serial_settings.flags |= ASYNC_SPD_CUST;
        kernel_serial_settings.custom_divisor = kernel_serial_settings.baud_base / speed;
    }
    if (ioctl(fd, TIOCSSERIAL, &kernel_serial_settings)) {
        printf("error %d from ioctl, %s\n", errno, strerror(errno));
        return -1;
    }
#endif
    return 0;
}

bool comm_init(comm_interface_t *i,
               const char *serial_device,
               unsigned baudrate,
               comm_rcv_cb_t rcv_cb)
{
    i->fd = open(serial_device, O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);
    printf("open\n");
    if (i->fd < 0) {
        printf("error %d opening %s: %s", errno, serial_device, strerror(errno));
        return false;
    }
    if (set_interface_attribs(i->fd, baudrate, 0) != 0) {
        return false;
    }

    i->rcv_cb = rcv_cb;
    serial_datagram_rcv_handler_init(&i->datagram_rcv_handler,
            i->rcv_datagram_buffer,
            sizeof(i->rcv_datagram_buffer),
            (serial_datagram_cb_t)datagram_rcv_cb,
            (void*)i);

    assert(pthread_mutex_init(&i->send_lock, NULL) == 0);

    return true;
}

#else // ChibiOS

void comm_init(comm_interface_t *i,
               BaseSequentialStream* serial_device,
               comm_rcv_cb_t rcv_cb)
{
    i->fd = serial_device;
    i->rcv_cb = rcv_cb;
    serial_datagram_rcv_handler_init(&i->datagram_rcv_handler,
            i->rcv_datagram_buffer,
            sizeof(i->rcv_datagram_buffer),
            (serial_datagram_cb_t)datagram_rcv_cb,
            (void*)i);

    chMtxObjectInit(&i->send_lock);
}

#endif


void comm_send(comm_interface_t *i,
               comm_msg_id_t msg_id,
               const void *msg,
               size_t size)
{
    assert(size <= ROS_INTERFACE_MTU);


#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    assert(pthread_mutex_lock(&i->send_lock) == 0);
#else // ChibiOS
    chMtxLock(&i->send_lock);
#endif

    static_assert(sizeof(comm_msg_id_t) == 2, "adjust msg id serialization");
    i->send_msg_buffer[0] = (uint8_t) (msg_id >>  0u);
    i->send_msg_buffer[1] = (uint8_t) (msg_id >>  8u);

    if (size != 0) {
        memcpy(&i->send_msg_buffer[sizeof(comm_msg_id_t)], msg, size);
    }

    size_t datagram_size = serial_datagram_buffer_wrap(
            i->send_msg_buffer,
            size + sizeof(comm_msg_id_t),
            i->send_datagram_buffer,
            sizeof(i->send_datagram_buffer));

    size_t bytes_written = 0;
    while (datagram_size - bytes_written > 0) {
#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
        int ret = write(i->fd, &i->send_datagram_buffer[bytes_written], datagram_size - bytes_written);
        if (ret == -1) {
            printf("write failed, %s\n", strerror(errno));
            break;
        }
#else // ChibiOS
        size_t ret = streamWrite(i->fd, &i->send_datagram_buffer[bytes_written], datagram_size - bytes_written);
#endif
        bytes_written += ret;
    }

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    tcdrain(i->fd); // wait until all data is sent out
    assert(pthread_mutex_unlock(&i->send_lock) == 0);
#else // ChibiOS
    chMtxUnlock(&i->send_lock);
#endif

}


void comm_receive(comm_interface_t *i)
{
    uint8_t buf[1];
#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    int len = read(i->fd, buf, sizeof(buf));
    if (len < 0) {
        printf("read failed %s\n", strerror(errno));
        return;
    }
#else // ChibiOS
    size_t len = streamRead(i->fd, buf, 1);
    if (len == 0) {
        chThdSleepMilliseconds(1); // queue is probably reset, avoid busy loop
    }
#endif

    int ret = serial_datagram_receive(&i->datagram_rcv_handler, buf, len);
    if (ret != SERIAL_DATAGRAM_RCV_NO_ERROR) {
#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
        printf("receive error\n");
#endif
    }
}
