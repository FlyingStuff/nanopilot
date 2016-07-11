#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include "../sumd.h"

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

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
        return 0;
}



int main(const int argc, const char **argv)
{
    if (argc != 2) {
        printf("usage: %s /dev/serialport\n", argv[0]);
        return -1;
    }
    const char *portname = argv[1];
    struct sumd_receiver_s receiver;
    sumd_receiver_init(&receiver);

    int fd = open (portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
            printf("error %d opening %s: %s", errno, portname, strerror (errno));
            return -1;
    }
    printf("open ok \n");
    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    printf("set interface ok\n");
    fcntl(fd, F_SETFL, 0); // blocking
    while (1) {
        char rc;
        if (read(fd, &rc, 1) != 1) {
            // sleep(1);
        } else {
            // printf("%c\n", rc);
            int ret = sumd_receive(&receiver, rc);
            if (ret == SUMD_RECEIVE_ERROR) {
                printf("error\n");
            }
            if (ret == SUMD_RECEIVE_COMPLETE) {
                if (receiver.no_signal_failsafe) {
                    printf("\nfailsafe: ");
                } else {
                    printf("\nvalid: ");
                }
                int i;
                for (i = 0; i < receiver.nb_channels; i++) {
                    printf("%dus  ", receiver.channel[i]/8);
                }
            }
        }
    }
    close(fd);
    return 0;
}


