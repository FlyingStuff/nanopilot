#include "rclcpp/rclcpp.hpp"
// #include <sensor_msgs/msg/imu.hpp>
#include <autopilot_msgs/msg/attitude_trajectory_setpoint.hpp>
#include <autopilot_msgs/msg/rate_control_setpoint.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <chrono>
using namespace std::chrono_literals;

#include <Eigen/Dense>

using namespace std::chrono_literals;

/* split a rotation such that:
 * rotation = rotation_twist * rotation_tilt
 * (first tilt, then twist)
 * where rotation_twist is around twist_axis and rotation_tilt is around a
 * perpendicular axis
 */
void rotation_split_tilt_twist(const Eigen::Quaterniond &rotation,
    const Eigen::Vector3d &twist_axis,
    Eigen::Quaterniond &rotation_tilt,
    Eigen::Quaterniond &rotation_twist)
{
    auto q = rotation.normalized();
    if (q.w() < 0) { // fix quaternion sign to have positive real part
        q.coeffs() = -q.coeffs();
    }
    auto t = twist_axis.normalized();
    double qv_dot_t = q.vec().dot(t);
    double s = 1/sqrt(qv_dot_t*qv_dot_t + q.w()*q.w());
    rotation_twist.w() = q.w() * s;
    auto b = qv_dot_t * s;
    rotation_twist.vec() = b * t;
    rotation_tilt = rotation_twist.conjugate()*q;
}

class AttitudeCtrl : public rclcpp::Node
{
public:
    AttitudeCtrl()
    : Node("AttitudeCtrl")
    {
        rclcpp::QoS pub_qos_settings(10);
        rclcpp::QoS sub_qos_settings(1);
        sub_qos_settings.best_effort();
        att_setpt_sub = this->create_subscription<autopilot_msgs::msg::AttitudeTrajectorySetpoint>(
            "attitude_setpoint", sub_qos_settings, std::bind(&AttitudeCtrl::att_setpt_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", sub_qos_settings, std::bind(&AttitudeCtrl::pose_cb, this, _1));
        ctrl_pub = this->create_publisher<autopilot_msgs::msg::RateControlSetpoint>("control", pub_qos_settings);

        control_timer = create_wall_timer(
            5ms, std::bind(&AttitudeCtrl::control_update, this));
    }


private:
    void control_update(void)
    {
        auto ctrl_msg = autopilot_msgs::msg::RateControlSetpoint();

        if (!pose_msg || !att_setpt_msg) {
            return;
        }
        if (pose_msg->header.frame_id != att_setpt_msg->header.frame_id) {
            auto clock = *(this->get_clock());
            RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 100, "setpt and pose frame mismatch");
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
        if (att_error.w() < 0) { // fix attitude error to have positive real part
            att_error.coeffs() = -att_error.coeffs();
        }
        Eigen::Vector3d twist_axis(0, 0, 1);
        Eigen::Quaterniond att_error_twist, att_error_tilt;
        rotation_split_tilt_twist(att_error, twist_axis, att_error_tilt, att_error_twist);

        Eigen::Vector3d att_error_vec;
        att_error_vec = 2*(att_error_tilt.vec() + att_error_twist.vec());
        // att_error_vec = 2*att_error.vec();

        Eigen::Vector3d rate_setpt = -K.cwiseProduct(att_error_vec);
        rate_setpt = rate_setpt.cwiseMin(max_rate_setpt).cwiseMax(-max_rate_setpt);
        ctrl_msg.header.stamp = pose_msg->header.stamp;
        ctrl_msg.rate_control_setpoint.x = rate_setpt[0];
        ctrl_msg.rate_control_setpoint.y = rate_setpt[1];
        ctrl_msg.rate_control_setpoint.z = rate_setpt[2];
        ctrl_msg.force = att_setpt_msg->force;
        ctrl_pub->publish(ctrl_msg);

        auto dt = this->now() - rclcpp::Time(pose_msg->header.stamp);
        if (dt > rclcpp::Duration(10ms)) {
            RCLCPP_WARN(this->get_logger(), "old pose data %f ms", (float)dt.nanoseconds()/1000/1000);
        }
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
