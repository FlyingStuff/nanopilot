#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// todo: these two lines are needed because of the Node being created with make shared
// http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

#include "rclcpp/rclcpp.hpp"
#include <autopilot_msgs/msg/attitude_trajectory_setpoint.hpp>
#include <autopilot_msgs/msg/position_trajectory_setpoint.hpp>
#include <autopilot_msgs/msg/rate_control_setpoint.hpp>
#include <autopilot_msgs/msg/rc_input.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

using namespace std::chrono_literals;


class AttitudeRemoteControlSetpoint : public rclcpp::Node
{
    enum class Mode {attitude, velocity, position};

public:
    AttitudeRemoteControlSetpoint()
    : Node("AttitudeRemoteControlSetpoint"), yaw_angle_q(1, 0, 0, 0)
    {
        rclcpp::QoS pub_qos_settings(10);
        // pub_qos_settings.best_effort();
        rclcpp::QoS sub_qos_settings(1);
        sub_qos_settings.best_effort();

        rc_in_sub = this->create_subscription<autopilot_msgs::msg::RCInput>(
            "rc_input", sub_qos_settings, std::bind(&AttitudeRemoteControlSetpoint::rc_in_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", sub_qos_settings, std::bind(&AttitudeRemoteControlSetpoint::pose_cb, this, _1));

        att_setpt_pub = this->create_publisher<autopilot_msgs::msg::AttitudeTrajectorySetpoint>("attitude_setpoint", pub_qos_settings);
        att_setpt_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("attitude_setpoint_pose", pub_qos_settings);
        pos_vel_setpt_pub = this->create_publisher<autopilot_msgs::msg::PositionTrajectorySetpoint>("position_setpoint", pub_qos_settings);
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

            double fwd_vel = 1*msg->channels[2];
            double right_vel = -1*msg->channels[1];
            double down_vel = -1*msg->channels[0];

            // RCLCPP_INFO(this->get_logger(), "ctrl att %f %f %f",
            //     roll_angle,
            //     pitch_angle,
            //     yaw_angle_q.z());

            // TODO better state-machine with transitions
            Mode mode;
            if (msg->channels[5] < -0.5) {
                mode = Mode::attitude;
            } else if (msg->channels[5] < 0.5) {
                mode = Mode::velocity;
            } else {
                if (pose_msg) {
                    mode = Mode::position;
                    if (prev_mode != Mode::position) {
                        hold_pos = pose_msg->pose.position;
                    }
                } else {
                    mode = Mode::velocity;
                }
            }

            if ((mode != Mode::position && mode != Mode::velocity) && (prev_mode == Mode::position || prev_mode == Mode::velocity)) {
                auto ctrl_msg = autopilot_msgs::msg::PositionTrajectorySetpoint();
                ctrl_msg.enable_control = false;
                pos_vel_setpt_pub->publish(ctrl_msg);
            }

            switch (mode) {
                case Mode::attitude: {
                    auto ctrl_msg = autopilot_msgs::msg::AttitudeTrajectorySetpoint();
                    ctrl_msg.header.frame_id = "NED";
                    ctrl_msg.header.stamp = msg->stamp;
                    ctrl_msg.attitude.w = attitude.w();
                    ctrl_msg.attitude.x = attitude.x();
                    ctrl_msg.attitude.y = attitude.y();
                    ctrl_msg.attitude.z = attitude.z();
                    ctrl_msg.acceleration.z = -thrust;
                    att_setpt_pub->publish(ctrl_msg);
                    auto pose_msg_setpt = geometry_msgs::msg::PoseStamped();
                    pose_msg_setpt.header.frame_id = "NED";
                    pose_msg_setpt.header.stamp = msg->stamp;
                    pose_msg_setpt.pose.orientation.w = attitude.w();
                    pose_msg_setpt.pose.orientation.x = attitude.x();
                    pose_msg_setpt.pose.orientation.y = attitude.y();
                    pose_msg_setpt.pose.orientation.z = attitude.z();
                    att_setpt_pose_pub->publish(pose_msg_setpt);
                    break;
                }

                case Mode::velocity: {
                    auto ctrl_msg = autopilot_msgs::msg::PositionTrajectorySetpoint();
                    ctrl_msg.header.frame_id = "NED";
                    ctrl_msg.header.stamp = msg->stamp;
                    ctrl_msg.position.x = NAN;
                    ctrl_msg.position.y = NAN;
                    ctrl_msg.position.z = NAN;
                    ctrl_msg.velocity.x = fwd_vel; // TODO transform body to global frame
                    ctrl_msg.velocity.y = right_vel;
                    ctrl_msg.velocity.z = down_vel;
                    // acceleration
                    // jerk
                    // snap
                    ctrl_msg.enable_control = true;
                    pos_vel_setpt_pub->publish(ctrl_msg);
                    break;
                }

                case Mode::position: {
                    auto ctrl_msg = autopilot_msgs::msg::PositionTrajectorySetpoint();
                    ctrl_msg.header.frame_id = "NED";
                    ctrl_msg.header.stamp = msg->stamp;
                    ctrl_msg.position.x = hold_pos.x;
                    ctrl_msg.position.y = hold_pos.y;
                    ctrl_msg.position.z = hold_pos.z;
                    ctrl_msg.velocity.x = 0;
                    ctrl_msg.velocity.y = 0;
                    ctrl_msg.velocity.z = 0;
                    // acceleration
                    // jerk
                    // snap
                    ctrl_msg.enable_control = true;
                    pos_vel_setpt_pub->publish(ctrl_msg);
                    break;
                }
            }

            prev_mode = mode;
        }
        prev_rc_in = msg;
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_msg = msg;
    }

    Eigen::Quaterniond yaw_angle_q;

    // ROS Subscribers
    rclcpp::Subscription<autopilot_msgs::msg::RCInput>::SharedPtr rc_in_sub;
    autopilot_msgs::msg::RCInput::SharedPtr prev_rc_in;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_msg;

    Mode prev_mode = Mode::attitude;
    geometry_msgs::msg::Point hold_pos;

    // ROS Publishers
    rclcpp::Publisher<autopilot_msgs::msg::AttitudeTrajectorySetpoint>::SharedPtr att_setpt_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr att_setpt_pose_pub;
    rclcpp::Publisher<autopilot_msgs::msg::PositionTrajectorySetpoint>::SharedPtr pos_vel_setpt_pub;

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
