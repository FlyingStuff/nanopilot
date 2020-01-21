#include <stdio.h>
#include <thread>
#include <chrono>
#include <string>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <autopilot_msgs/msg/rc_input.hpp>
#include "autopilot_msgs/msg/rate_control_setpoint.hpp"
#include "autopilot_msgs/msg/actuator_positions_stamped.hpp"
#include "autopilot_msgs/msg/actuator_positions.hpp"
#include "autopilot_msgs/srv/send_msgpack_config.hpp"
#include "autopilot_msgs/msg/time_sync_stat.hpp"
#include <chrono>
#include "comm.h"
#include "msg.h"
using std::placeholders::_1;
using namespace std::chrono_literals;


#include <sys/time.h>

using namespace std::chrono_literals;

struct ping_s {
    uint32_t idx;
    struct timeval tv;
};




template<typename MsgType>
void deserialize_and_publish(const uint8_t *msg, size_t len, msgbus::Topic<MsgType> &topic)
{
    auto deserializer = nop::Deserializer<nop::BufferReader>(msg, len);
    MsgType val;
    auto status = deserializer.Read(&val);
    if (status) {
        topic.publish(val);
    }
}


class LowLevelControllerInterface : public rclcpp::Node
{
public:
    LowLevelControllerInterface()
    : Node("LowLevelControllerInterface") {
        this->declare_parameter("baud");
        this->declare_parameter("port");
        uint32_t baudrate = this->get_parameter("baud").as_int();
        const std::string portname = this->get_parameter("port").as_string();

        auto cb = [this](comm_msg_id_t msg_id, const uint8_t *msg, size_t len){this->rx_handler(msg_id, msg, len);};
        if (!comm_init(&m_interface, portname.c_str(), baudrate, cb)) {
            exit(-1);
        }
        rclcpp::QoS pub_qos_settings(10);
        // pub_qos_settings.best_effort();
        m_time_sync_pub = this->create_publisher<autopilot_msgs::msg::TimeSyncStat>("time_sync", pub_qos_settings);
        m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", pub_qos_settings);
        m_mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("magnetometer", pub_qos_settings);
        m_rcinput_pub = this->create_publisher<autopilot_msgs::msg::RCInput>("rc_input", pub_qos_settings);
        m_ping_latency_pub = this->create_publisher<std_msgs::msg::Float64>("ping_latency", pub_qos_settings);
        m_rate_ctrl_setpoint_pub = this->create_publisher<geometry_msgs::msg::Vector3>("rate_ctrl/setpoint", pub_qos_settings);
        m_rate_ctrl_measured_pub = this->create_publisher<geometry_msgs::msg::Vector3>("rate_ctrl/measured", pub_qos_settings);
        m_rate_ctrl_output_pub = this->create_publisher<geometry_msgs::msg::Vector3>("rate_ctrl/output", pub_qos_settings);
        m_output_armed_pub = this->create_publisher<std_msgs::msg::Bool>("output_armed", pub_qos_settings);
        m_ap_in_control_pub = this->create_publisher<std_msgs::msg::Bool>("ap_in_control", pub_qos_settings);
        m_ap_latency_pub = this->create_publisher<std_msgs::msg::UInt64>("ap_latency", pub_qos_settings);
        m_output_pub = this->create_publisher<autopilot_msgs::msg::ActuatorPositions>("output", pub_qos_settings);

        rclcpp::QoS sub_qos_settings(1);
        sub_qos_settings.best_effort();
        m_ap_ctrl_sub = this->create_subscription<autopilot_msgs::msg::RateControlSetpoint>("control", sub_qos_settings, std::bind(&LowLevelControllerInterface::control_cb, this, _1));

        auto rx_thd = std::thread(rx_thd_fn, &m_interface);
        rx_thd.detach();
        auto publish_thd = std::thread(std::bind(&LowLevelControllerInterface::publish_thd_fn, this));
        publish_thd.detach();
        m_ping_timer = this->create_wall_timer(101ms, [this](){this->send_ping();});
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
                struct timeval tv_now;
                gettimeofday(&tv_now, NULL);
                struct ping_s ping;
                memcpy(&ping, msg, sizeof(ping));
                float ms = (double) (tv_now.tv_usec - ping.tv.tv_usec) / 1000 +
                 (double) (tv_now.tv_sec - ping.tv.tv_sec) * 1000;
                // RCLCPP_INFO(this->get_logger(), "ping %d: size %d, latency %f ms", ping.idx, sizeof(struct ping_s), ms);
                if (ms > 10) {
                    RCLCPP_WARN(this->get_logger(), "ping %d: size %d, high latency %f ms", ping.idx, sizeof(struct ping_s), ms);
                }
                if (ping.idx > m_ping_rcv_idx+1) {
                    RCLCPP_WARN(this->get_logger(), "ping %d: %d packets lost", ping.idx, ping.idx - (m_ping_rcv_idx+1));
                }
                m_ping_rcv_idx = ping.idx;
                m_ping_latency_buf.publish(ms);
            }
            break;

        case RosInterfaceCommMsgID::LOG:
            std::cout.write((char*)msg, len);
            break;

        case RosInterfaceCommMsgID::HEARTBEAT:
            // printf("HEARTBEAT\n");
            break;

        case RosInterfaceCommMsgID::TIME:
        {
            auto now = this->now();
            auto message_latency = now - m_timesync_trigger_time;
            if (message_latency < rclcpp::Duration(50ms)) {
                assert(len == sizeof(uint64_t));
                uint64_t timestamp;
                memcpy(&timestamp, msg, sizeof(timestamp));
                auto llc_time = rclcpp::Time(timestamp, RCL_ROS_TIME);
                // m_timesync_trigger_time = rclcpp::Time(std::chrono::steady_clock.now().time_since_epoch(), RCL_ROS_TIME);
                rclcpp::Duration offset = m_timesync_trigger_time - llc_time;
                rclcpp::Duration uncertainty(0);
                if (m_timesync_gpio_fd >= 0) {
                    // we are using a GPIO to sync time
                    uncertainty = m_timesync_uncertainty;
                } else {
                    // using TIME_SYNC message to sync time
                    uncertainty = message_latency;
                }
                rclcpp::Duration adjust = offset - m_timestamp_offset;
                if (abs(adjust.nanoseconds()) > rclcpp::Duration(500ms).nanoseconds()) {
                    RCLCPP_WARN(this->get_logger(), "time jump %f s", (float)adjust.nanoseconds()/1e9);
                    m_timestamp_offset = offset;
                } else {
                    if (uncertainty < rclcpp::Duration(10us)) {
                        uncertainty = rclcpp::Duration(10us);
                    }
                    float alpha = 0.1;
                    m_timestamp_offset = m_timestamp_offset + adjust * alpha;
                }
                auto message = autopilot_msgs::msg::TimeSyncStat();
                message.header.stamp = now;
                message.timestamp = timestamp;
                message.offset = m_timestamp_offset.nanoseconds();
                message.adjust = adjust.nanoseconds();
                m_time_sync_pub->publish(message);
            }
            break;
        }

        case RosInterfaceCommMsgID::RC_INPUT:
            deserialize_and_publish(msg, len, m_rc_input_buf);
            break;
        case RosInterfaceCommMsgID::ACTUATOR_OUTPUT:
            deserialize_and_publish(msg, len, m_actuator_output_buf);
            break;
        case RosInterfaceCommMsgID::IMU:
            deserialize_and_publish(msg, len, m_imu_buf);
            break;
        case RosInterfaceCommMsgID::MAGNETOMETER:
            deserialize_and_publish(msg, len, m_mag_buf);
            break;
        case RosInterfaceCommMsgID::RATE_CTRL_SETPOINT_RPY:
            deserialize_and_publish(msg, len, m_rate_ctrl_setpoint_buf);
            break;
        case RosInterfaceCommMsgID::RATE_CTRL_MEASURED_RPY:
            deserialize_and_publish(msg, len, m_rate_ctrl_measured_buf);
            break;
        case RosInterfaceCommMsgID::RATE_CTRL_OUTPUT_RPY:
            deserialize_and_publish(msg, len, m_rate_ctrl_output_buf);
            break;
        case RosInterfaceCommMsgID::OUTPUT_IS_ARMED:
            deserialize_and_publish(msg, len, m_output_armed_buf);
            break;
        case RosInterfaceCommMsgID::AP_IN_CONTROL:
            deserialize_and_publish(msg, len, m_ap_in_control_buf);
            break;
        case RosInterfaceCommMsgID::AP_LATENCY:
            deserialize_and_publish(msg, len, m_ap_latency_buf);
            break;
        default:
            std::string msg_str((char*)msg, len);
            printf("unknown rx msg %d, len: %d, %s\n", (int)msg_id, (int)len, msg_str.c_str());
            break;
        }
    }

    void control_cb(const autopilot_msgs::msg::RateControlSetpoint::SharedPtr msg)
    {
        struct ap_ctrl_s ctrl;
        ctrl.timestamp = (rclcpp::Time(msg->header.stamp) - m_timestamp_offset).nanoseconds();
        ctrl.rate_setpoint_rpy[0] = msg->rate_control_setpoint.x;
        ctrl.rate_setpoint_rpy[1] = msg->rate_control_setpoint.y;
        ctrl.rate_setpoint_rpy[2] = msg->rate_control_setpoint.z;
        static_assert(autopilot_msgs::msg::ActuatorPositions::MAX_NB_ACTUATORS == MAX_NB_ACTUATORS, "message definition mismatch");
        for (unsigned i=0; i < msg->actuators.actuators.size(); i++) {
            ctrl.direct_output[i] = msg->actuators.actuators[i];
        }
        ctrl.feed_forward_torque_rpy[0] = msg->feed_forward_torque.x;
        ctrl.feed_forward_torque_rpy[1] = msg->feed_forward_torque.y;
        ctrl.feed_forward_torque_rpy[2] = msg->feed_forward_torque.z;
        ctrl.force_xyz[0] = msg->force.x;
        ctrl.force_xyz[1] = msg->force.y;
        ctrl.force_xyz[2] = msg->force.z;

        static uint8_t buffer[1000];
        auto serializer = nop::Serializer<nop::BufferWriter>(buffer, sizeof(buffer));
        serializer.Write(ctrl);
        comm_send(&m_interface, RosInterfaceCommMsgID::AP_CONTROL, buffer, serializer.writer().size());
    }

    void publish_thd_fn()
    {
        std::vector<msgbus::SubscriberBase*> sub_list;

        auto rc_input_buf_sub = msgbus::subscribe(m_rc_input_buf);
        sub_list.push_back(&rc_input_buf_sub);
        auto actuator_output_sub = msgbus::subscribe(m_actuator_output_buf);
        sub_list.push_back(&actuator_output_sub);
        auto imu_sub = msgbus::subscribe(m_imu_buf);
        sub_list.push_back(&imu_sub);
        auto mag_sub = msgbus::subscribe(m_mag_buf);
        sub_list.push_back(&mag_sub);
        auto rate_ctrl_setpoint_sub = msgbus::subscribe(m_rate_ctrl_setpoint_buf);
        sub_list.push_back(&rate_ctrl_setpoint_sub);
        auto rate_ctrl_measured_sub = msgbus::subscribe(m_rate_ctrl_measured_buf);
        sub_list.push_back(&rate_ctrl_measured_sub);
        auto rate_ctrl_output_sub = msgbus::subscribe(m_rate_ctrl_output_buf);
        sub_list.push_back(&rate_ctrl_output_sub);
        auto output_armed_sub = msgbus::subscribe(m_output_armed_buf);
        sub_list.push_back(&output_armed_sub);
        auto ap_in_control_sub = msgbus::subscribe(m_ap_in_control_buf);
        sub_list.push_back(&ap_in_control_sub);
        auto ping_latency_sub = msgbus::subscribe(m_ping_latency_buf);
        sub_list.push_back(&ping_latency_sub);
        auto ap_latency_sub = msgbus::subscribe(m_ap_latency_buf);
        sub_list.push_back(&ap_latency_sub);
        while (1) {
            msgbus::wait_for_update_on_any(sub_list.begin(), sub_list.end());
            struct timeval start;
            gettimeofday(&start, NULL);

            const char *topic = "";
            try {
                if (rc_input_buf_sub.has_update()) {
                    topic = "rc_input_buf";
                    auto val = rc_input_buf_sub.get_value();
                    auto message = autopilot_msgs::msg::RCInput();
                    message.stamp = rclcpp::Time(val.timestamp, RCL_SYSTEM_TIME);
                    auto nb_channels = std::min(static_cast<uint32_t>(val.nb_channels), message.MAX_NB_CHANNELS);
                    for (uint32_t i=0; i < nb_channels; i++) {
                        message.channels.push_back(val.channel[i]);
                    }
                    message.signal = !val.no_signal;
                    m_rcinput_pub->publish(message);
                } else
                if (actuator_output_sub.has_update()) {
                    topic = "actuator_output";
                    auto val = actuator_output_sub.get_value();
                    auto message = autopilot_msgs::msg::ActuatorPositions();
                    static_assert(autopilot_msgs::msg::ActuatorPositions::MAX_NB_ACTUATORS == MAX_NB_ACTUATORS, "message definition mismatch");
                    for (auto a: val) {
                        message.actuators.push_back(a);
                    }
                    m_output_pub->publish(message);
                } else
                if (imu_sub.has_update()) {
                    topic = "imu";
                    auto imu = imu_sub.get_value();
                    auto message = sensor_msgs::msg::Imu();
                    message.header.stamp = rclcpp::Time(imu.timestamp, RCL_SYSTEM_TIME) + m_timestamp_offset;
                    message.header.frame_id = "imu";
                    message.orientation.x = imu.accumulated_angle.x;
                    message.orientation.y = imu.accumulated_angle.y;
                    message.orientation.z = imu.accumulated_angle.z;
                    message.orientation.w = imu.accumulated_angle.w;
                    message.orientation_covariance = {{0}};
                    message.angular_velocity.x = imu.angular_rate[0];
                    message.angular_velocity.y = imu.angular_rate[1];
                    message.angular_velocity.z = imu.angular_rate[2];
                    message.angular_velocity_covariance = {{0}};
                    message.linear_acceleration.x = imu.linear_acceleration[0];
                    message.linear_acceleration.y = imu.linear_acceleration[1];
                    message.linear_acceleration.z = imu.linear_acceleration[2];
                    message.linear_acceleration_covariance = {{0}};
                    m_imu_pub->publish(message);
                } else
                if (mag_sub.has_update()) {
                    topic = "mag";
                    magnetometer_sample_t mag = mag_sub.get_value();
                    auto message = sensor_msgs::msg::MagneticField();
                    message.header.stamp = rclcpp::Time(mag.timestamp, RCL_SYSTEM_TIME) + m_timestamp_offset;
                    message.header.frame_id = "imu";
                    message.magnetic_field.x = mag.magnetic_field[0];
                    message.magnetic_field.y = mag.magnetic_field[1];
                    message.magnetic_field.z = mag.magnetic_field[2];
                    message.magnetic_field_covariance = {{0}};
                    m_mag_pub->publish(message);
                } else
                if(rate_ctrl_setpoint_sub.has_update()) {
                    topic = "rate_ctrl_setpoint";
                    auto val = rate_ctrl_setpoint_sub.get_value();
                    auto message = geometry_msgs::msg::Vector3();
                    message.x = val[0];
                    message.y = val[1];
                    message.z = val[2];
                    m_rate_ctrl_setpoint_pub->publish(message);
                } else
                if(rate_ctrl_measured_sub.has_update()) {
                    topic = "rate_ctrl_measured";
                    auto val = rate_ctrl_measured_sub.get_value();
                    auto message = geometry_msgs::msg::Vector3();
                    message.x = val[0];
                    message.y = val[1];
                    message.z = val[2];
                    m_rate_ctrl_measured_pub->publish(message);
                } else
                if(rate_ctrl_output_sub.has_update()) {
                    topic = "rate_ctrl_output";
                    auto val = rate_ctrl_output_sub.get_value();
                    auto message = geometry_msgs::msg::Vector3();
                    message.x = val[0];
                    message.y = val[1];
                    message.z = val[2];
                    m_rate_ctrl_output_pub->publish(message);
                } else
                if (output_armed_sub.has_update()) {
                    topic = "output_armed";
                    auto val = output_armed_sub.get_value();
                    auto message = std_msgs::msg::Bool();
                    message.data = val;
                    m_output_armed_pub->publish(message);
                } else
                if (ap_in_control_sub.has_update()) {
                    topic = "ap_in_control";
                    auto val = ap_in_control_sub.get_value();
                    auto message = std_msgs::msg::Bool();
                    message.data = val;
                    m_ap_in_control_pub->publish(message);
                } else
                if (ping_latency_sub.has_update()) {
                    topic = "ping_latency";
                    auto val = ping_latency_sub.get_value();
                    auto message = std_msgs::msg::Float64();
                    message.data = val;
                    m_ping_latency_pub->publish(message);
                } else
                if (ap_latency_sub.has_update()) {
                    topic = "ap_latency";
                    auto val = ap_latency_sub.get_value();
                    auto message = std_msgs::msg::UInt64();
                    message.data = val;
                    m_ap_latency_pub->publish(message);
                }
            } catch(rclcpp::exceptions::RCLError const& e) {
                std::cout << "Message: " << e.what() << "\n";
                std::cout << "Type:    " << typeid(e).name() << "\n";
                std::cout << "Topic: " << topic << "\n";
            }

            struct timeval stop;
            gettimeofday(&stop, NULL);
            float ms = (double) (stop.tv_usec - start.tv_usec) / 1000 +
                 (double) (stop.tv_sec - start.tv_sec) * 1000;
            if (ms > 2) {
                std::cout << "slow publish: " << topic << " time: " << ms << " ms\n";
            }
        }
    }


    static void rx_thd_fn(comm_interface_t *interface) {
        while (1) {
            comm_receive(interface);
        }
    }

    void send_ping(void) {
        if (m_ping_idx > m_ping_rcv_idx+1) {
            RCLCPP_WARN(this->get_logger(), "ping %d: no reply", m_ping_idx);
        }
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
        if (m_timesync_gpio_fd >= 0) {
            // use GPIO to sync time
            auto before = this->now();
            if (write(m_timesync_gpio_fd, "1", 1) == 1) {
                auto after = this->now();
                // todo lock
                m_timesync_uncertainty = after - before;
                m_timesync_trigger_time = this->now();
            }
            if (write(m_timesync_gpio_fd, "0", 1) != 1) {
                // todo
            }
        } else {
            m_timesync_trigger_time = this->now();
            comm_send(&m_interface, RosInterfaceCommMsgID::TIME_SYNC, NULL, 0);
        }
    }

    rclcpp::TimerBase::SharedPtr m_ping_timer;
    rclcpp::TimerBase::SharedPtr m_timesync_timer;
    int m_timesync_gpio_fd{-1};
    rclcpp::Time m_timesync_trigger_time{{0, RCL_ROS_TIME}};
    rclcpp::Duration m_timesync_uncertainty{-1};
    rclcpp::Duration m_timestamp_offset{0};
    rclcpp::Service<autopilot_msgs::srv::SendMsgpackConfig>::SharedPtr m_param_set_srv;
    uint64_t m_ping_idx{0};
    uint64_t m_ping_rcv_idx{0};
    comm_interface_t m_interface;

    rclcpp::Publisher<autopilot_msgs::msg::RCInput>::SharedPtr m_rcinput_pub;
    msgbus::Topic<struct rc_input_s> m_rc_input_buf;
    rclcpp::Publisher<autopilot_msgs::msg::TimeSyncStat>::SharedPtr m_time_sync_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_ping_latency_pub;
    msgbus::Topic<float> m_ping_latency_buf;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;
    msgbus::Topic<imu_sample_t> m_imu_buf;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr m_mag_pub;
    msgbus::Topic<magnetometer_sample_t> m_mag_buf;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_rate_ctrl_setpoint_pub;
    msgbus::Topic<std::array<float, 3> > m_rate_ctrl_setpoint_buf;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_rate_ctrl_measured_pub;
    msgbus::Topic<std::array<float, 3> > m_rate_ctrl_measured_buf;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_rate_ctrl_output_pub;
    msgbus::Topic<std::array<float, 3> > m_rate_ctrl_output_buf;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_output_armed_pub;
    msgbus::Topic<bool> m_output_armed_buf;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_ap_in_control_pub;
    msgbus::Topic<bool> m_ap_in_control_buf;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr m_ap_latency_pub;
    msgbus::Topic<uint64_t> m_ap_latency_buf;
    rclcpp::Publisher<autopilot_msgs::msg::ActuatorPositions>::SharedPtr m_output_pub;
    msgbus::Topic<std::vector<float> > m_actuator_output_buf;
    rclcpp::Subscription<autopilot_msgs::msg::RateControlSetpoint>::SharedPtr m_ap_ctrl_sub;
};



int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);

    auto llc_if_node = std::make_shared<LowLevelControllerInterface>();

    rclcpp::spin(llc_if_node);
    rclcpp::shutdown();
    return 0;
}
