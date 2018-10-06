#include <stdio.h>
#include <thread>
#include <chrono>
#include <string>
#include <string.h>
#include <iostream>

#include "comm.h"
#include "msg.h"

#include <sys/time.h>

comm_interface_t interface;

struct ping_s {
    uint32_t idx;
    struct timeval tv;
} ;

void rx_cb(comm_msg_id_t msg_id, const uint8_t *msg, size_t len)
{
    switch (msg_id) {
    case RosInterfaceCommMsgID::PING:
        comm_send(&interface, RosInterfaceCommMsgID::PONG, msg, len);
        break;
    case RosInterfaceCommMsgID::PONG:
        if (len == sizeof(struct ping_s)) {
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            struct ping_s ping;
            memcpy(&ping, msg, sizeof(ping));
            float ms = (double) (tv_now.tv_usec - ping.tv.tv_usec) / 1000 +
             (double) (tv_now.tv_sec - ping.tv.tv_sec) * 1000;
            printf("ping %u: size %u latency %f ms\n", ping.idx, sizeof(struct ping_s),  ms);
        }
        break;
    case RosInterfaceCommMsgID::HEARTBEAT:
        printf("HEARTBEAT\n");
        break;
    case RosInterfaceCommMsgID::TIME:
        
        break;
    default:
        std::string msg_str((char*)msg, len);
        printf("unknown rx msg %d, len: %d, %s\n", (int)msg_id, (int)len, msg_str.c_str());
        break;
    }
}



int main(const int argc, const char **argv)
{
    if (argc != 3) {
        printf("usage: %s /dev/serialport baudrate\n", argv[0]);
        return -1;
    }
    const char *portname = argv[1];
    int baudrate = atoi(argv[2]);

    if (!comm_init(&interface, portname, baudrate, rx_cb)) {
        return -1;
    }

    std::thread ping_thread([&](){
        for (uint32_t i=0; true; i++) {
            struct ping_s ping;
            ping.idx = i;
            gettimeofday(&ping.tv, NULL);
            comm_send(&interface, RosInterfaceCommMsgID::PING, &ping, sizeof(ping));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    while (1) {
        // printf("recv\n");
        comm_receive(&interface);
    }
    ping_thread.join();
}
