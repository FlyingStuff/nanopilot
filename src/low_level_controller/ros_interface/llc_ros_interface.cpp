#include <stdio.h>
#include <thread>
#include <chrono>
#include <string>
#include <string.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <autopilot_msgs/msg/rc_input.hpp>
#include "autopilot_msgs/srv/send_msgpack_config.hpp"
#include <chrono>
#include "comm.h"
#include "msg.h"

#include <sys/time.h>

using namespace std::chrono_literals;

struct ping_s {
    uint32_t idx;
    struct timeval tv;
};

class LowLevelControllerInterface : public rclcpp::Node
{
public:
    LowLevelControllerInterface(const std::string portname, uint32_t baudrate)
    : Node("LowLevelControllerInterface") {

        auto cb = [this](comm_msg_id_t msg_id, const uint8_t *msg, size_t len){this->rx_handler(msg_id, msg, len);};
        if (!comm_init(&m_interface, portname.c_str(), baudrate, cb)) {
            exit(-1);
        }

        // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
        // // Depth represents how many messages to store in history when the history policy is KEEP_LAST.
        // custom_qos_profile.depth = 1;
        // // The reliability policy can be reliable, meaning that the underlying transport layer will try
        // // ensure that every message gets received in order, or best effort, meaning that the transport
        // // makes no guarantees about the order or reliability of delivery.
        // custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        // // The history policy determines how messages are saved until the message is taken by the reader.
        // // KEEP_ALL saves all messages until they are taken.
        // // KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
        // // parameter.
        // custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        m_timestamp_pub = this->create_publisher<std_msgs::msg::UInt64>("timestamp");
        m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu");
        m_rcinput_pub = this->create_publisher<autopilot_msgs::msg::RCInput>("rc_input");
        m_latency_pub = this->create_publisher<std_msgs::msg::Float64>("ping_latency");
        m_rate_ctrl_setpoint_pub = this->create_publisher<geometry_msgs::msg::Vector3>("rate_ctrl_setpoint");
        m_rate_ctrl_measured_pub = this->create_publisher<geometry_msgs::msg::Vector3>("rate_ctrl_measured");
        m_rate_ctrl_output_pub = this->create_publisher<geometry_msgs::msg::Vector3>("rate_ctrl_output");

        auto rx_thd = std::thread(rx_thd_fn, &m_interface);
        rx_thd.detach();
        m_ping_timer = this->create_wall_timer(1000ms, [this](){this->send_ping();});
        m_timesync_timer = this->create_wall_timer(100ms, [this](){this->trigger_timesync();});

        m_param_set_srv = this->create_service<autopilot_msgs::srv::SendMsgpackConfig>("send_config",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<autopilot_msgs::srv::SendMsgpackConfig::Request> request,
                const std::shared_ptr<autopilot_msgs::srv::SendMsgpackConfig::Response> response){
                    this->handle_parameter_set_service(request_header, request, response);
                });

    }

private:

    void rx_handler(comm_msg_id_t msg_id, const uint8_t *msg, size_t len)
    {
        switch (msg_id) {
        case RosInterfaceCommMsgID::PING:
            comm_send(&m_interface, RosInterfaceCommMsgID::PONG, msg, len);
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
                // std::cout << "ping " << ping.idx << ": size " << sizeof(struct ping_s) << " latency " << ms << " ms" << std::endl;
                RCLCPP_INFO(this->get_logger(), "ping %d: size %d, latency %f ms", ping.idx, sizeof(struct ping_s), ms);
                if (ping.idx > prev_ping_idx+1) {
                    // std::cout << ping.idx - (prev_ping_idx+1) << " packets lost" << std::endl;
                    RCLCPP_WARN(this->get_logger(), "ping %d: %d packets lost", ping.idx, ping.idx - (prev_ping_idx+1));
                }
                prev_ping_idx = ping.idx;
            }
            break;

        case RosInterfaceCommMsgID::LOG:
            std::cout.write((char*)msg, len);
            break;

        case RosInterfaceCommMsgID::HEARTBEAT:
            // printf("HEARTBEAT\n");
            break;

        case RosInterfaceCommMsgID::TIME:
            if (len == sizeof(uint64_t)) {
                uint64_t timestamp;
                memcpy(&timestamp, msg, sizeof(timestamp));
                auto message = std_msgs::msg::UInt64();
                message.data = timestamp;
                m_timestamp_pub->publish(message);
            }
            break;

        case RosInterfaceCommMsgID::RC_INPUT:
        {
            auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
            rc_input_s val;
            deserializer.Read(&val);
            auto message = autopilot_msgs::msg::RCInput();
            message.stamp = rclcpp::Time(val.timestamp, RCL_SYSTEM_TIME);
            auto nb_channels = std::min(static_cast<uint32_t>(val.nb_channels), message.MAX_NB_CHANNELS);
            for (uint32_t i=0; i < nb_channels; i++) {
                message.channels.push_back(val.channel[i]);
            }
            message.signal = !val.no_signal;
            m_rcinput_pub->publish(message);
            break;
        }

        case RosInterfaceCommMsgID::ACTUATOR_OUTPUT:
        {
            // auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
            // std::vector<float> val;
            // deserializer.Read(&val);
            // std::cout << "out ";
            // for (auto o: val) {
            //     std::cout << o << " ";
            // }
            // std::cout << std::endl;
            break;
        }

        case RosInterfaceCommMsgID::IMU:
        {
            auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
            rate_gyro_sample_t gyro;
            deserializer.Read(&gyro);
            accelerometer_sample_t acc;
            deserializer.Read(&acc);
            auto message = sensor_msgs::msg::Imu();
            message.header.stamp = rclcpp::Time(gyro.timestamp, RCL_SYSTEM_TIME);
            message.header.frame_id = "imu";
            message.orientation.x = gyro.accumulated_angle.x;
            message.orientation.y = gyro.accumulated_angle.y;
            message.orientation.z = gyro.accumulated_angle.z;
            message.orientation.w = gyro.accumulated_angle.w;
            message.orientation_covariance = {0};
            message.angular_velocity.x = gyro.rate[0];
            message.angular_velocity.y = gyro.rate[1];
            message.angular_velocity.z = gyro.rate[2];
            message.angular_velocity_covariance = {0};
            message.linear_acceleration.x = acc.acceleration[0];
            message.linear_acceleration.y = acc.acceleration[1];
            message.linear_acceleration.z = acc.acceleration[2];
            message.linear_acceleration_covariance = {0};
            m_imu_pub->publish(message);
            break;
        }

        case RosInterfaceCommMsgID::MAGNETOMETER:
        {
            auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
            magnetometer_sample_t mag;
            deserializer.Read(&mag);
            auto message = sensor_msgs::msg::MagneticField();
            message.header.stamp = rclcpp::Time(mag.timestamp, RCL_SYSTEM_TIME);
            message.header.frame_id = "imu";
            message.magnetic_field.x = mag.magnetic_field[0];
            message.magnetic_field.y = mag.magnetic_field[1];
            message.magnetic_field.z = mag.magnetic_field[2];
            message.magnetic_field_covariance = {0};
            m_mag_pub->publish(message);
            break;
        }

        case RosInterfaceCommMsgID::RATE_CTRL_SETPOINT_RPY:
        {
            auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
            std::array<float, 3> val;
            deserializer.Read(&val);
            auto message = geometry_msgs::msg::Vector3();
            message.x = val[0];
            message.y = val[1];
            message.z = val[2];
            m_rate_ctrl_setpoint_pub->publish(message);
            break;
        }

        case RosInterfaceCommMsgID::RATE_CTRL_MEASURED_RPY:
        {
            auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
            std::array<float, 3> val;
            deserializer.Read(&val);
            auto message = geometry_msgs::msg::Vector3();
            message.x = val[0];
            message.y = val[1];
            message.z = val[2];
            m_rate_ctrl_measured_pub->publish(message);
            break;
        }

        case RosInterfaceCommMsgID::RATE_CTRL_OUTPUT_RPY:
        {
            auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
            std::array<float, 3> val;
            deserializer.Read(&val);
            auto message = geometry_msgs::msg::Vector3();
            message.x = val[0];
            message.y = val[1];
            message.z = val[2];
            m_rate_ctrl_output_pub->publish(message);
            break;
        }

        default:
            std::string msg_str((char*)msg, len);
            printf("unknown rx msg %d, len: %d, %s\n", (int)msg_id, (int)len, msg_str.c_str());
            break;
        }
    }


    static void rx_thd_fn(comm_interface_t *interface) {
        while (1) {
            comm_receive(interface);
        }
    }

    void send_ping(void) {
        struct ping_s ping;
        ping.idx = m_ping_idx++;
        gettimeofday(&ping.tv, NULL);
        comm_send(&m_interface, RosInterfaceCommMsgID::PING, &ping, sizeof(ping));
    }


    void handle_parameter_set_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<autopilot_msgs::srv::SendMsgpackConfig::Request> request,
      const std::shared_ptr<autopilot_msgs::srv::SendMsgpackConfig::Response> response)
    {
      (void)request_header;
      RCLCPP_INFO(this->get_logger(),"parameters received");
      comm_send(&m_interface, RosInterfaceCommMsgID::SET_PARAMETERS, request->msgpack_config.data(), request->msgpack_config.size());
      response->success = true;
    }


    void trigger_timesync(void) {
    }

    rclcpp::TimerBase::SharedPtr m_ping_timer;
    rclcpp::TimerBase::SharedPtr m_timesync_timer;
    rclcpp::Service<autopilot_msgs::srv::SendMsgpackConfig>::SharedPtr m_param_set_srv;
    uint64_t m_ping_idx{0};
    uint64_t m_ping_rcv_idx{0};
    comm_interface_t m_interface;
    rclcpp::Publisher<autopilot_msgs::msg::RCInput>::SharedPtr m_rcinput_pub;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr m_timestamp_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_latency_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_mag_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_rate_ctrl_setpoint_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_rate_ctrl_measured_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_rate_ctrl_output_pub;
};



int main(const int argc, const char **argv)
{
    if (argc != 3) {
        printf("usage: %s /dev/serialport baudrate\n", argv[0]);
        return -1;
    }
    const char *portname = argv[1];
    int baudrate = atoi(argv[2]);

    rclcpp::init(argc, argv);

    auto llc_if_node = std::make_shared<LowLevelControllerInterface>(portname, baudrate);

    rclcpp::spin(llc_if_node);
    rclcpp::shutdown();
    return 0;
}
