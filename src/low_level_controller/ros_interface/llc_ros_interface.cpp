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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autopilot_msgs/msg/rc_input.hpp>
#include "autopilot_msgs/msg/attitude_controller_status.hpp"
#include "autopilot_msgs/msg/attitude_trajectory_setpoint.hpp"
#include "autopilot_msgs/msg/actuator_positions_stamped.hpp"
#include "autopilot_msgs/msg/actuator_positions.hpp"
#include "autopilot_msgs/msg/control_status.hpp"
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
        m_ap_latency_pub = this->create_publisher<std_msgs::msg::Float64>("ap_latency", pub_qos_settings);
        m_output_pub = this->create_publisher<autopilot_msgs::msg::ActuatorPositionsStamped>("output", pub_qos_settings);
        m_ctrl_status_pub = this->create_publisher<autopilot_msgs::msg::ControlStatus>("control_status", pub_qos_settings);
        m_attitude_filter_out_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("attitude_filter", pub_qos_settings);
        m_ctrl_attitude_status_pub = this->create_publisher<autopilot_msgs::msg::AttitudeControllerStatus>("attitude_control_status", pub_qos_settings);

        rclcpp::QoS sub_qos_settings(1);
        sub_qos_settings.best_effort();
        m_att_ctrl_sub = this->create_subscription<autopilot_msgs::msg::AttitudeTrajectorySetpoint>("attitude_control_setpoint", sub_qos_settings, std::bind(&LowLevelControllerInterface::att_control_cb, this, _1));
        m_att_ref_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("attitude_reference", sub_qos_settings, std::bind(&LowLevelControllerInterface::att_ref_cb, this, _1));

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
        case RosInterfaceCommMsgID::AP_LATENCY:
            deserialize_and_publish(msg, len, m_ap_latency_buf);
            break;
        case RosInterfaceCommMsgID::CONTROL_STATUS:
            deserialize_and_publish(msg, len, m_ctrl_status_buf);
            break;
        case RosInterfaceCommMsgID::ATTITUDE_FILTER_OUTPUT:
            deserialize_and_publish(msg, len, m_attitude_filter_out_buf);
            break;
        case RosInterfaceCommMsgID::CONTOLLER_ATTITUDE_STATUS:
            deserialize_and_publish(msg, len, m_ctrl_attitude_status_buf);
            break;
        default:
            std::string msg_str((char*)msg, len);
            printf("unknown rx msg %d, len: %d, %s\n", (int)msg_id, (int)len, msg_str.c_str());
            break;
        }
    }

    void att_control_cb(const autopilot_msgs::msg::AttitudeTrajectorySetpoint::SharedPtr msg)
    {
        attitude_controller_input_t ctrl;
        ctrl.timestamp = (rclcpp::Time(msg->header.stamp) - m_timestamp_offset).nanoseconds();
        ctrl.attitude.x = msg->attitude.x;
        ctrl.attitude.y = msg->attitude.y;
        ctrl.attitude.z = msg->attitude.z;
        ctrl.attitude.w = msg->attitude.w;
        ctrl.angular_rate[0] = msg->angular_rate.x;
        ctrl.angular_rate[1] = msg->angular_rate.y;
        ctrl.angular_rate[2] = msg->angular_rate.z;
        ctrl.angular_acceleration[0] = msg->angular_acceleration.x;
        ctrl.angular_acceleration[1] = msg->angular_acceleration.y;
        ctrl.angular_acceleration[2] = msg->angular_acceleration.z;
        ctrl.acceleration[0] = msg->acceleration.x;
        ctrl.acceleration[1] = msg->acceleration.y;
        ctrl.acceleration[2] = msg->acceleration.z;

        static uint8_t buffer[1000];
        auto serializer = nop::Serializer<nop::BufferWriter>(buffer, sizeof(buffer));
        serializer.Write(ctrl);
        comm_send(&m_interface, RosInterfaceCommMsgID::CONTOLLER_ATTITUDE_SETPT, buffer, serializer.writer().size());
    }

    void att_ref_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        external_attitude_reference_t ref;
        ref.attitude_reference.w = msg->pose.orientation.w;
        ref.attitude_reference.x = msg->pose.orientation.x;
        ref.attitude_reference.y = msg->pose.orientation.y;
        ref.attitude_reference.z = msg->pose.orientation.z;
        ref.timestamp = (rclcpp::Time(msg->header.stamp) - m_timestamp_offset).nanoseconds();
        static uint8_t buffer[1000];
        auto serializer = nop::Serializer<nop::BufferWriter>(buffer, sizeof(buffer));
        serializer.Write(ref);
        comm_send(&m_interface, RosInterfaceCommMsgID::ATTITUDE_FILTER_REFERENCE, buffer, serializer.writer().size());
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
        auto ping_latency_sub = msgbus::subscribe(m_ping_latency_buf);
        sub_list.push_back(&ping_latency_sub);
        auto ap_latency_sub = msgbus::subscribe(m_ap_latency_buf);
        sub_list.push_back(&ap_latency_sub);
        auto ctrl_status_sub = msgbus::subscribe(m_ctrl_status_buf);
        sub_list.push_back(&ctrl_status_sub);
        auto attitude_filter_out_sub = msgbus::subscribe(m_attitude_filter_out_buf);
        sub_list.push_back(&attitude_filter_out_sub);
        auto ctrl_attitude_status_sub = msgbus::subscribe(m_ctrl_attitude_status_buf);
        sub_list.push_back(&ctrl_attitude_status_sub);
        while (1) {
            msgbus::wait_for_update_on_any(sub_list.begin(), sub_list.end());
            struct timeval start;
            gettimeofday(&start, NULL);

            const char *topic = "";
            try {
                if (rc_input_buf_sub.has_update()) {
                    topic = "rc_input";
                    auto val = rc_input_buf_sub.get_value();
                    auto message = autopilot_msgs::msg::RCInput();
                    message.stamp = rclcpp::Time(val.timestamp, RCL_SYSTEM_TIME)  + m_timestamp_offset;
                    message.roll = val.roll;
                    message.pitch = val.pitch;
                    message.yaw = val.yaw;
                    message.throttle = val.throttle;
                    auto nb_channels = std::min(static_cast<uint32_t>(val.channel_raw_count), message.MAX_NB_CHANNELS);
                    for (uint32_t i=0; i < nb_channels; i++) {
                        message.channels.push_back(val.channel_raw[i]);
                    }
                    message.signal = val.signal;
                    message.rssi = val.rssi;
                    m_rcinput_pub->publish(message);
                }
                else if (actuator_output_sub.has_update()) {
                    topic = "actuator_output";
                    auto val = actuator_output_sub.get_value();
                    auto message = autopilot_msgs::msg::ActuatorPositionsStamped();
                    static_assert(autopilot_msgs::msg::ActuatorPositions::MAX_NB_ACTUATORS == NB_ACTUATORS, "message definition mismatch");
                    for (unsigned i = 0; i < val.actuators.actuators_len; i++) {
                        message.actuators.actuators.push_back(val.actuators.actuators[i]);
                    }
                    message.stamp = rclcpp::Time(val.timestamp, RCL_SYSTEM_TIME)  + m_timestamp_offset;
                    m_output_pub->publish(message);
                }
                else if (imu_sub.has_update()) {
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
                }
                else if (mag_sub.has_update()) {
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
                }
                else if (ping_latency_sub.has_update()) {
                    topic = "ping_latency";
                    auto val = ping_latency_sub.get_value();
                    auto message = std_msgs::msg::Float64();
                    message.data = val;
                    m_ping_latency_pub->publish(message);
                }
                else if (ap_latency_sub.has_update()) {
                    topic = "ap_latency";
                    auto val = ap_latency_sub.get_value();
                    auto message = std_msgs::msg::Float64();
                    message.data = val;
                    m_ap_latency_pub->publish(message);
                }
                else if (ctrl_status_sub.has_update()) {
                    topic = "ctrl_status";
                    auto val = ctrl_status_sub.get_value();
                    auto message = autopilot_msgs::msg::ControlStatus();
                    message.mode = val.mode;
                    m_ctrl_status_pub->publish(message);
                }
                else if (attitude_filter_out_sub.has_update()) {
                    topic = "attitude_filter_out";
                    auto val = attitude_filter_out_sub.get_value();
                    auto message = geometry_msgs::msg::PoseStamped();
                    message.header.stamp = rclcpp::Time(val.timestamp, RCL_SYSTEM_TIME) + m_timestamp_offset;
                    message.header.frame_id = "NED"; // TODO
                    message.pose.orientation.w = val.attitude.w;
                    message.pose.orientation.x = val.attitude.x;
                    message.pose.orientation.y = val.attitude.y;
                    message.pose.orientation.z = val.attitude.z;
                    m_attitude_filter_out_pub->publish(message);
                }
                else if (ctrl_attitude_status_sub.has_update()) {
                    topic = "ctrl_attitude_status";
                    auto val = ctrl_attitude_status_sub.get_value();
                    auto message = autopilot_msgs::msg::AttitudeControllerStatus();
                    message.header.stamp = rclcpp::Time(val.timestamp, RCL_SYSTEM_TIME) + m_timestamp_offset;
                    message.angular_rate_ref.x = val.angular_rate_ref[0];
                    message.angular_rate_ref.y = val.angular_rate_ref[1];
                    message.angular_rate_ref.z = val.angular_rate_ref[2];
                    message.torque.x = val.torque[0];
                    message.torque.y = val.torque[1];
                    message.torque.z = val.torque[2];
                    m_ctrl_attitude_status_pub->publish(message);
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_ap_latency_pub;
    msgbus::Topic<float> m_ap_latency_buf;
    rclcpp::Publisher<autopilot_msgs::msg::ActuatorPositionsStamped>::SharedPtr m_output_pub;
    msgbus::Topic<actuators_stamped_t> m_actuator_output_buf;
    rclcpp::Publisher<autopilot_msgs::msg::ControlStatus>::SharedPtr m_ctrl_status_pub;
    msgbus::Topic<control_status_t> m_ctrl_status_buf;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_attitude_filter_out_pub;
    msgbus::Topic<attitude_filter_output_t> m_attitude_filter_out_buf;
    rclcpp::Publisher<autopilot_msgs::msg::AttitudeControllerStatus>::SharedPtr m_ctrl_attitude_status_pub;
    msgbus::Topic<attitude_controller_status_t> m_ctrl_attitude_status_buf;
    rclcpp::Subscription<autopilot_msgs::msg::AttitudeTrajectorySetpoint>::SharedPtr m_att_ctrl_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_att_ref_sub;
};



int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);

    auto llc_if_node = std::make_shared<LowLevelControllerInterface>();

    rclcpp::spin(llc_if_node);
    rclcpp::shutdown();
    return 0;
}
