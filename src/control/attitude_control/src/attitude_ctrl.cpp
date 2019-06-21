#include "rclcpp/rclcpp.hpp"
// #include <sensor_msgs/msg/imu.hpp>
#include <autopilot_msgs/msg/attitude_trajectory_setpoint.hpp>
#include <autopilot_msgs/msg/rate_control_setpoint.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <chrono>

#include <Eigen/Dense>

using namespace std::chrono_literals;


class AttitudeCtrl : public rclcpp::Node
{
public:
    AttitudeCtrl()
    : Node("AttitudeCtrl")
    {
        att_setpt_sub = this->create_subscription<autopilot_msgs::msg::AttitudeTrajectorySetpoint>(
            "attitude_setpoint", std::bind(&AttitudeCtrl::att_setpt_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", std::bind(&AttitudeCtrl::pose_cb, this, _1));
        ctrl_pub = this->create_publisher<autopilot_msgs::msg::RateControlSetpoint>("control");

        control_timer = create_wall_timer(
            20ms, std::bind(&AttitudeCtrl::control_update, this));
    }


private:

    void control_update(void)
    {
        // https://doi.org/10.3929/ethz-a-009970340

        auto ctrl_msg = autopilot_msgs::msg::RateControlSetpoint();

        if (!pose_msg || !att_setpt_msg) {
            return;
        }
        double roll_time_cst = 0.1; // [s]
        double pitch_time_cst = 0.1; // [s]
        double yaw_time_cst = 0.5; // [s]
        Eigen::Vector3d K(1/roll_time_cst, 1/pitch_time_cst, 1/yaw_time_cst);
        Eigen::Vector3d max_rate_setpt(2, 2, 2);
        auto setpt_body_to_nav = Eigen::Quaterniond(att_setpt_msg->orientation.w,
            att_setpt_msg->orientation.x,
            att_setpt_msg->orientation.y,
            att_setpt_msg->orientation.z);
        auto estimate_body_to_nav = Eigen::Quaterniond(pose_msg->pose.orientation.w,
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z);

        auto att_error = (setpt_body_to_nav.conjugate()*estimate_body_to_nav).normalized();
        Eigen::Vector3d att_error_vec;
        if (att_error.w() < 0) {
            att_error_vec = -2*att_error.normalized().vec();
        } else {
            att_error_vec = 2*att_error.normalized().vec();
        }
        Eigen::Vector3d rate_setpt = -K.cwiseProduct(att_error_vec);
        rate_setpt = rate_setpt.cwiseMin(max_rate_setpt).cwiseMax(-max_rate_setpt);
        ctrl_msg.header.stamp = pose_msg->header.stamp;
        ctrl_msg.rate_control_setpoint.x = rate_setpt[0];
        ctrl_msg.rate_control_setpoint.y = rate_setpt[1];
        ctrl_msg.rate_control_setpoint.z = rate_setpt[2];
        ctrl_msg.force = att_setpt_msg->force;
        ctrl_pub->publish(ctrl_msg);
        // RCLCPP_INFO(this->get_logger(), "ctrl '%f %f %f'", rate_setpt[0], rate_setpt[1], rate_setpt[2]);
    }



    void att_setpt_cb(const autopilot_msgs::msg::AttitudeTrajectorySetpoint::SharedPtr msg)
    {
        att_setpt_msg = msg;
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_msg = msg;
    }

    // ROS Subscribers
    rclcpp::Subscription<autopilot_msgs::msg::AttitudeTrajectorySetpoint>::SharedPtr att_setpt_sub;
    autopilot_msgs::msg::AttitudeTrajectorySetpoint::SharedPtr att_setpt_msg;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_msg;

    // ROS Publishers
    rclcpp::Publisher<autopilot_msgs::msg::RateControlSetpoint>::SharedPtr ctrl_pub;

    rclcpp::TimerBase::SharedPtr control_timer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudeCtrl>());
    rclcpp::shutdown();
    return 0;
}
