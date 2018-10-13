#include <stdio.h>
#include <thread>
#include <chrono>
#include <string>
#include <string.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/u_int64.hpp>


#include "comm.h"
#include "msg.h"

#include <sys/time.h>

comm_interface_t interface;

struct ping_s {
    uint32_t idx;
    struct timeval tv;
};


class TimePublisher : public rclcpp::Node
{
public:
    TimePublisher()
    : Node("TimePublisher") {
        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
        // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
        custom_qos_profile.depth = 1;
        // The reliability policy can be reliable, meaning that the underlying transport layer will try
        // ensure that every message gets received in order, or best effort, meaning that the transport
        // makes no guarantees about the order or reliability of delivery.
        custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        // The history policy determines how messages are saved until the message is taken by the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
        // parameter.
        custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("llc_timestamp", custom_qos_profile);
    }
    void publish_time(uint64_t timestamp) {
        auto message = std_msgs::msg::UInt64();
        message.data = timestamp;
        // RCLCPP_INFO(this->get_logger(), "Publishing Time");
        this->publisher_->publish(message);
    }
private:
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
};

std::shared_ptr<TimePublisher> time_pub;


void rx_cb(comm_msg_id_t msg_id, const uint8_t *msg, size_t len)
{
    switch (msg_id) {
    case RosInterfaceCommMsgID::PING:
        comm_send(&interface, RosInterfaceCommMsgID::PONG, msg, len);
        break;
    case RosInterfaceCommMsgID::PONG:
        if (len == sizeof(struct ping_s)) {
            static uint32_t prev_ping_idx = 0;
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            struct ping_s ping;
            memcpy(&ping, msg, sizeof(ping));
            float ms = (double) (tv_now.tv_usec - ping.tv.tv_usec) / 1000 +
             (double) (tv_now.tv_sec - ping.tv.tv_sec) * 1000;
            printf("ping %u: size %u latency %f ms\n", ping.idx, sizeof(struct ping_s),  ms);
            if (ping.idx > prev_ping_idx+1) {
                printf("%d packets lost\n", ping.idx - (prev_ping_idx+1));
            }
            prev_ping_idx = ping.idx;
        }
        break;
    case RosInterfaceCommMsgID::HEARTBEAT:
        printf("HEARTBEAT\n");
        break;
    case RosInterfaceCommMsgID::TIME:
        if (len == sizeof(uint64_t)) {
            uint64_t timestamp;
            memcpy(&timestamp, msg, sizeof(timestamp));
            time_pub->publish_time(timestamp);
        }
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
            printf("sending ping %d\n", i);
            gettimeofday(&ping.tv, NULL);
            comm_send(&interface, RosInterfaceCommMsgID::PING, &ping, sizeof(ping));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    rclcpp::init(argc, argv);

    time_pub = std::make_shared<TimePublisher>();
    while (1) {
        comm_receive(&interface);
    }
    ping_thread.join();
    rclcpp::shutdown();
}
