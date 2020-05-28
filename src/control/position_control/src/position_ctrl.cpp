#include "rclcpp/rclcpp.hpp"
#include <autopilot_msgs/msg/position_trajectory_setpoint.hpp>
#include <autopilot_msgs/msg/acceleration_trajectory_setpoint.hpp>
#include "autopilot_msgs/msg/control_status.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;
#include <iostream>
#include <chrono>
#include <pid/pid.hpp>
#include <Eigen/Dense>
#include <string.h>
#include <math.h>
#include <stdio.h>

using namespace std::chrono_literals;


class PositionCtrl : public rclcpp::Node
{
public:
    PositionCtrl()
    : Node("PositionCtrl")
    {
        position_setpt_sub = this->create_subscription<autopilot_msgs::msg::PositionTrajectorySetpoint>(
            "position_setpoint", 1, std::bind(&PositionCtrl::position_setpt_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 1, std::bind(&PositionCtrl::pose_cb, this, _1));
        twist_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "twist", 1, std::bind(&PositionCtrl::twist_cb, this, _1));
        control_status_sub = this->create_subscription<autopilot_msgs::msg::ControlStatus>(
            "control_status", 1, std::bind(&PositionCtrl::control_active_cb, this, _1));

        acc_ctrl_pub = this->create_publisher<autopilot_msgs::msg::AccelerationTrajectorySetpoint>("acceleration_setpoint", 10);
        vel_setpt_pub = this->create_publisher<geometry_msgs::msg::Vector3>("velocity_setpoint", 10);

        this->parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
        while (!this->parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        sub_params = this->parameters_client->on_parameter_event(
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
        {
            on_parameter_event(event);
        });

        parameters_init();
        int update_rate = 50; // [Hz]
        control_timer = create_wall_timer(
            1.0s/update_rate, std::bind(&PositionCtrl::control_update, this));
        pid_velocity_x.set_frequency(update_rate);
        pid_velocity_y.set_frequency(update_rate);
        pid_velocity_z.set_frequency(update_rate);
    }

private:

    void control_update(void)
    {

        if (current_pose_msg && position_setpt_msg && current_twist_msg)
        {
            if (!position_setpt_msg->enable_control || !control_active) {
                pid_velocity_x.reset_integral();
                pid_velocity_y.reset_integral();
                pid_velocity_z.reset_integral();
                return;
            }
            if (current_pose_msg->header.frame_id != position_setpt_msg->header.frame_id) {
                auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);
                RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 100, "setpt and pose frame mismatch");
                return;
            }
            if (current_twist_msg->header.frame_id != position_setpt_msg->header.frame_id) {
                auto clock = rclcpp::Clock(RCL_SYSTEM_TIME);
                RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 100, "setpt and twist frame mismatch");
                return;
            }

            // TODO check time of messages
            double error_position_x = 0;
            double error_position_y = 0;
            double error_position_z = 0;
            if (isfinite(position_setpt_msg->position.x)) {
                if (isfinite(current_pose_msg->pose.position.x)) {
                    error_position_x = current_pose_msg->pose.position.x - position_setpt_msg->position.x;
                } else {
                    auto clock = *(this->get_clock());
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 100, "No x position available");
                }
            }
            if (isfinite(position_setpt_msg->position.y)) {
                if (isfinite(current_pose_msg->pose.position.y)) {
                    error_position_y = current_pose_msg->pose.position.y - position_setpt_msg->position.y;
                } else {
                    auto clock = *(this->get_clock());
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 100, "No y position available");
                }
            }
            if (isfinite(position_setpt_msg->position.z)) {
                if (isfinite(current_pose_msg->pose.position.z)) {
                    error_position_z = current_pose_msg->pose.position.z - position_setpt_msg->position.z;
                } else {
                    auto clock = *(this->get_clock());
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 100, "No z position available");
                }
            }

            double velocity_setpoint_x = -kp_position_x * error_position_x + position_setpt_msg->velocity.x;
            double velocity_setpoint_y = -kp_position_y * error_position_y + position_setpt_msg->velocity.y;
            double velocity_setpoint_z = -kp_position_z * error_position_z + position_setpt_msg->velocity.z;

            // TODO limit velocity as a vector
            // zero higher derivatives if limited

            double error_velocity_x = current_twist_msg->twist.linear.x - velocity_setpoint_x;
            double error_velocity_y = current_twist_msg->twist.linear.y - velocity_setpoint_y;
            double error_velocity_z = current_twist_msg->twist.linear.z - velocity_setpoint_z;

            double acc_setpt_x = pid_velocity_x.process(error_velocity_x) + position_setpt_msg->acceleration.x;
            double acc_setpt_y = pid_velocity_y.process(error_velocity_y) + position_setpt_msg->acceleration.y;
            double acc_setpt_z = pid_velocity_z.process(error_velocity_z) + position_setpt_msg->acceleration.z;

            auto vel_setpt_msg = geometry_msgs::msg::Vector3();
            vel_setpt_msg.x = velocity_setpoint_x;
            vel_setpt_msg.y = velocity_setpoint_y;
            vel_setpt_msg.z = velocity_setpoint_z;
            vel_setpt_pub->publish(vel_setpt_msg);

            auto acc_ctrl_msg = autopilot_msgs::msg::AccelerationTrajectorySetpoint();
            acc_ctrl_msg.header.stamp = current_twist_msg->header.stamp;
            acc_ctrl_msg.header.frame_id = position_setpt_msg->header.frame_id;
            acc_ctrl_msg.acceleration.x = acc_setpt_x;
            acc_ctrl_msg.acceleration.y = acc_setpt_y;
            acc_ctrl_msg.acceleration.z = acc_setpt_z;
            acc_ctrl_msg.jerk = position_setpt_msg->jerk;
            acc_ctrl_msg.snap = position_setpt_msg->snap;
            acc_ctrl_pub->publish(acc_ctrl_msg);
        }
    }

    double get_numeric_parameters(const char *name)
    {

        rclcpp::Parameter parameter = this->get_parameter(name);
        if (parameter.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
        {
            RCLCPP_INFO(this->get_logger(), "Parameter %s = %lf", name, parameter.as_double());
            return parameter.as_double();
        }
        if (parameter.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        {
            RCLCPP_INFO(this->get_logger(), "Parameter %s = %d", name, parameter.as_int());
            return parameter.as_int();
        }

        return 0;
    }
    void trigger_parameters_update()
    {
        RCLCPP_INFO(this->get_logger(), "--------------------------");
        this->kp_position_x = get_numeric_parameters("px_kp");
        this->kp_position_y = get_numeric_parameters("py_kp");
        this->kp_position_z = get_numeric_parameters("pz_kp");
        this->pid_velocity_x.set_gains(get_numeric_parameters("vx_kp"),
                                     get_numeric_parameters("vx_ki"),
                                     get_numeric_parameters("vx_kd"));

        this->pid_velocity_y.set_gains(get_numeric_parameters("vy_kp"),
                                     get_numeric_parameters("vy_ki"),
                                     get_numeric_parameters("vy_kd"));

        this->pid_velocity_z.set_gains(get_numeric_parameters("vz_kp"),
                                     get_numeric_parameters("vz_ki"),
                                     get_numeric_parameters("vz_kd"));

    }

    void on_parameter_event(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
        if (event->changed_parameters.size() > 0)
        {
            trigger_parameters_update();
        }
    }

    void position_setpt_cb(const autopilot_msgs::msg::PositionTrajectorySetpoint::SharedPtr msg)
    {
        position_setpt_msg = msg;
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_msg = msg;
    }

    void twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_twist_msg = msg;
    }

    void control_active_cb(const autopilot_msgs::msg::ControlStatus::SharedPtr msg)
    {
        if (msg->mode == autopilot_msgs::msg::ControlStatus::MODE_AP) {
            control_active = true;
        } else {
            control_active = false;
        }
    }

    void parameters_init(){

        this->declare_parameter("px_kp", 1.0);
        this->declare_parameter("py_kp", 1.0);
        this->declare_parameter("pz_kp", 1.0);

        this->declare_parameter("vx_kp", 1.0);
        this->declare_parameter("vx_ki", 0.0);
        this->declare_parameter("vx_kd", 0.0);

        this->declare_parameter("vy_kp", 1.0);
        this->declare_parameter("vy_ki", 0.0);
        this->declare_parameter("vy_kd", 0.0);

        this->declare_parameter("vz_kp", 1.0);
        this->declare_parameter("vz_ki", 0.0);
        this->declare_parameter("vz_kd", 0.0);

        trigger_parameters_update();

        RCLCPP_INFO(this->get_logger(), "Parameters correctly set on the server");

    }

    // // ROS Subscribers
    rclcpp::Subscription<autopilot_msgs::msg::PositionTrajectorySetpoint>::SharedPtr position_setpt_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr sub_params;
    rclcpp::Subscription<autopilot_msgs::msg::ControlStatus>::SharedPtr control_status_sub;

    autopilot_msgs::msg::PositionTrajectorySetpoint::SharedPtr position_setpt_msg;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_msg;
    geometry_msgs::msg::TwistStamped::SharedPtr current_twist_msg;

    // // ROS Publishers
    rclcpp::Publisher<autopilot_msgs::msg::AccelerationTrajectorySetpoint>::SharedPtr acc_ctrl_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_setpt_pub;

    bool control_active{false};

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::SyncParametersClient::SharedPtr parameters_client;

    PID pid_velocity_x;
    PID pid_velocity_y;
    PID pid_velocity_z;

    double kp_position_x;
    double kp_position_y;
    double kp_position_z;

};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionCtrl>());
    rclcpp::shutdown();
    return 0;
}
