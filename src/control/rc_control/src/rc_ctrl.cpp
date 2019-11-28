#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// todo: these two lines are needed because of the Node being created with make shared
// http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

#include "rclcpp/rclcpp.hpp"
#include <autopilot_msgs/msg/attitude_trajectory_setpoint.hpp>
#include <autopilot_msgs/msg/rate_control_setpoint.hpp>
#include <autopilot_msgs/msg/rc_input.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <chrono>

#include <Eigen/Dense>

using namespace std::chrono_literals;


class AttitudeRemoteControlSetpoint : public rclcpp::Node
{
public:
    AttitudeRemoteControlSetpoint()
    : Node("AttitudeRemoteControlSetpoint"), yaw_angle_q(1, 0, 0, 0)
    {
        rclcpp::QoS qos_settings(1);
        qos_settings.best_effort();
        rc_in_sub = this->create_subscription<autopilot_msgs::msg::RCInput>(
            "rc_input", qos_settings, std::bind(&AttitudeRemoteControlSetpoint::rc_in_cb, this, _1));
        // rate_setpt_sub = this->create_publisher<autopilot_msgs::msg::RateControlSetpoint>("rate_setpoint");

        att_setpt_pub = this->create_publisher<autopilot_msgs::msg::AttitudeTrajectorySetpoint>("attitude_setpoint", qos_settings);
        att_setpt_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("attitude_setpoint_pose", qos_settings);
    }


private:

    void rc_in_cb(const autopilot_msgs::msg::RCInput::SharedPtr msg)
    {
        if (!msg->signal) {
            return;
        }
        if (prev_rc_in) {
            double dt = (rclcpp::Time(msg->stamp) - rclcpp::Time(prev_rc_in->stamp)).seconds();
            double yaw_rate_gain = 1;
            double da = yaw_rate_gain * -msg->channels[3]*dt;
            yaw_angle_q = (yaw_angle_q * Eigen::Quaterniond(cos(da/2), 0, 0, sin(da/2))).normalized();
            double pitch_angle = -M_PI*30/180*msg->channels[2];
            auto pitch_angle_q = Eigen::Quaterniond(cos(pitch_angle/2), 0, sin(pitch_angle/2), 0);
            double roll_angle = -M_PI*30/180*msg->channels[1];
            auto roll_angle_q = Eigen::Quaterniond(cos(roll_angle/2), sin(roll_angle/2), 0, 0);
            auto attitude = yaw_angle_q * roll_angle_q * pitch_angle_q;
            double thrust = (msg->channels[0] + 1)/2;

            // RCLCPP_INFO(this->get_logger(), "ctrl att %f %f %f",
            //     roll_angle,
            //     pitch_angle,
            //     yaw_angle_q.z());

            auto ctrl_msg = autopilot_msgs::msg::AttitudeTrajectorySetpoint();
            ctrl_msg.header.frame_id = "NED";
            ctrl_msg.header.stamp = msg->stamp;
            ctrl_msg.orientation.w = attitude.w();
            ctrl_msg.orientation.x = attitude.x();
            ctrl_msg.orientation.y = attitude.y();
            ctrl_msg.orientation.z = attitude.z();
            ctrl_msg.force.z = -thrust;
            att_setpt_pub->publish(ctrl_msg);
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.frame_id = "NED";
            pose_msg.header.stamp = msg->stamp;
            pose_msg.pose.orientation.w = attitude.w();
            pose_msg.pose.orientation.x = attitude.x();
            pose_msg.pose.orientation.y = attitude.y();
            pose_msg.pose.orientation.z = attitude.z();
            att_setpt_pose_pub->publish(pose_msg);

        }
        prev_rc_in = msg;
    }

    Eigen::Quaterniond yaw_angle_q;

    // ROS Subscribers
    rclcpp::Subscription<autopilot_msgs::msg::RCInput>::SharedPtr rc_in_sub;
    autopilot_msgs::msg::RCInput::SharedPtr prev_rc_in;

    // ROS Publishers
    rclcpp::Publisher<autopilot_msgs::msg::AttitudeTrajectorySetpoint>::SharedPtr att_setpt_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr att_setpt_pose_pub;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudeRemoteControlSetpoint>());
    rclcpp::shutdown();
    return 0;
}
