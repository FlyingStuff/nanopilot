#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <chrono>
using namespace std::chrono_literals;

class sub : public rclcpp::Node
{
public:
    sub()
    : Node("sub")
    {
        rclcpp::QoS sub_qos_settings(1);
        sub_qos_settings.best_effort();
        _sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "timing_test", sub_qos_settings, std::bind(&sub::pose_cb, this, _1));
    }

private:
    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // std::cerr << '.';
        auto dt = this->now() - rclcpp::Time(msg->header.stamp);
        // if (dt > rclcpp::Duration(1ms)) {
            RCLCPP_WARN(this->get_logger(), "%f ms", (float)dt.nanoseconds()/1000/1000);
        // }
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sub>());
    rclcpp::shutdown();
    return 0;
}
