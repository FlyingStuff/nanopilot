#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#if defined(__linux__)
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif


int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf("error %d from tcgetattr, %s\n", errno, strerror(errno));
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
    if (ioctl(fd, TIOCSSERIAL, &kernel_serial_settings)) {
        printf("error %d from ioctl, %s\n", errno, strerror(errno));
        return -1;
    }
#endif
    return 0;
}

#include <sys/time.h>
#include "statistics.h"

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

    statistics_t time_stats;
    statistics_reset(&time_stats);

    int i;
    for (i=0; i < 1000; i++) {

        struct timeval  tv1, tv2;
        gettimeofday(&tv1, NULL);

        char x = 'a'+i%30;
        int ret = write(fd, &x, 1);
        if (ret == -1) {
            printf("write failed %d", errno);
            continue;
        } else {
            // printf("write done\n");
        }
        char buf;
        ret = read(fd, &buf, sizeof(buf));
        if (ret == -1) {
            printf("read failed %d", errno);
            continue;
        } else {
            if (buf != x) {
                printf("read_wrong_char %c, expected %c\n", buf, x);
                continue;
            }
            // printf("read %c\n", buf);
        }
        gettimeofday(&tv2, NULL);
        float ms = (double) (tv2.tv_usec - tv1.tv_usec) / 1000 +
         (double) (tv2.tv_sec - tv1.tv_sec) * 1000;
        // printf ("Total time = %f ms\n", ms);
        statistics_add_sample(&time_stats, ms);
    }

    printf("_____________\n");

    printf("%d samples\n", statistics_get_nb_samples(&time_stats));
    printf("mean %f ms\n", statistics_get_mean(&time_stats));
    printf("stddev %f ms\n", statistics_get_stddev(&time_stats));
    printf("min %f ms\n", statistics_get_min(&time_stats));
    printf("max %f ms\n", statistics_get_max(&time_stats));

    close(fd);
    return 0;
}


