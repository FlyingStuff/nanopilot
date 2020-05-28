#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// todo: these two lines are needed because of the Node being created with make shared
// http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

#include "rclcpp/rclcpp.hpp"
#include <autopilot_msgs/msg/attitude_trajectory_setpoint.hpp>
#include <autopilot_msgs/msg/acceleration_trajectory_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <Eigen/Dense>

#include <string>


/** Find the quaternion that rotates v1 -> v2 and is the shortest rotation
 */
Eigen::Quaterniond shortest_rotation_quaternion(
    const Eigen::Vector3d &v1,
    const Eigen::Vector3d &v2)
{
    auto v1n = v1.normalized();
    auto v2n = v2.normalized();
    auto c = (v1n + v2n).normalized(); // TODO handle case where norm is zero
    auto qv = v1n.cross(c);
    Eigen::Quaterniond q(v1n.dot(c), qv[0], qv[1], qv[2]);
    return q;
}



class MCForceAllocNode : public rclcpp::Node
{
public:
    MCForceAllocNode()
    : Node("MCForceAlloc")
    {
        acc_setpt_sub = this->create_subscription<autopilot_msgs::msg::AccelerationTrajectorySetpoint>(
            "acceleration_setpoint", 1, std::bind(&MCForceAllocNode::acc_setpt_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 1, std::bind(&MCForceAllocNode::pose_cb, this, _1));
        ctrl_pub = this->create_publisher<autopilot_msgs::msg::AttitudeTrajectorySetpoint>("attitude_setpoint", 10);
    }

private:

    void accel_to_attitude(const Eigen::Vector3d &accel,
        const Eigen::Quaterniond &current_att_b_to_I,
        const Eigen::Quaterniond &yaw_ref_att_Y_to_I,
        Eigen::Quaterniond &desired_att,
        Eigen::Vector3d &acceleration)
    {
        const Eigen::Vector3d gravity(0, 0, 9.81);
        double hover_thrust = 0.5;
        const Eigen::Vector3d accel_I = (accel - gravity)*(hover_thrust/(9.81));

        // see Attitude Setpoint Generation in doc folder
        Eigen::Vector3d accel_Y = yaw_ref_att_Y_to_I.conjugate().toRotationMatrix() * accel_I;
        Eigen::Vector3d accel_d(0, 0, -1); // up
        auto desired_to_Y = shortest_rotation_quaternion(accel_d, accel_Y);
        desired_att = yaw_ref_att_Y_to_I * desired_to_Y;

        acceleration = current_att_b_to_I.conjugate().toRotationMatrix() * accel_I;
    }

    void acc_setpt_cb(autopilot_msgs::msg::AccelerationTrajectorySetpoint::SharedPtr msg)
    {
        if (pose_msg) {
            rclcpp::Time now = msg->header.stamp;
            if (fabs((now - rclcpp::Time(pose_msg->header.stamp)).seconds()) > 0.1) {
                RCLCPP_WARN(this->get_logger(), "pose message old %f", (now - rclcpp::Time(pose_msg->header.stamp)).seconds());
            }
            if (pose_msg->header.frame_id != msg->header.frame_id) {
                auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);
                RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 100, "setpt and pose frame mismatch");
            }

            Eigen::Quaterniond current_att(pose_msg->pose.orientation.w,
                pose_msg->pose.orientation.x,
                pose_msg->pose.orientation.y,
                pose_msg->pose.orientation.z);

            Eigen::Quaterniond yaw_ref_att = current_att;

            Eigen::Vector3d accel(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
            Eigen::Quaterniond att_d;
            Eigen::Vector3d acceleration_b;

            accel_to_attitude(accel, current_att, yaw_ref_att, att_d, acceleration_b);

            autopilot_msgs::msg::AttitudeTrajectorySetpoint setpt;
            setpt.header.frame_id = msg->header.frame_id;
            setpt.header.stamp = msg->header.stamp;
            setpt.attitude.w = att_d.w();
            setpt.attitude.x = att_d.x();
            setpt.attitude.y = att_d.y();
            setpt.attitude.z = att_d.z();
            setpt.angular_rate.x = 0;
            setpt.angular_rate.y = 0;
            setpt.angular_rate.z = 0;
            setpt.angular_acceleration.x = 0;
            setpt.angular_acceleration.y = 0;
            setpt.angular_acceleration.z = 0;
            setpt.acceleration.x = acceleration_b[0];
            setpt.acceleration.y = acceleration_b[1];
            setpt.acceleration.z = acceleration_b[2];
            ctrl_pub->publish(setpt);
        }
    }

    void pose_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_msg = msg;
    }

    // ROS Subscribers
    rclcpp::Subscription<autopilot_msgs::msg::AccelerationTrajectorySetpoint>::SharedPtr acc_setpt_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_msg;
    // ROS Publishers
    rclcpp::Publisher<autopilot_msgs::msg::AttitudeTrajectorySetpoint>::SharedPtr ctrl_pub;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MCForceAllocNode>());
    rclcpp::shutdown();
    return 0;
}
