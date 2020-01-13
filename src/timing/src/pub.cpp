#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <chrono>
using namespace std::chrono_literals;

class pub : public rclcpp::Node
{
public:
    pub()
    : Node("pub")
    {
        rclcpp::QoS pub_qos_settings(10);
        pub_qos_settings.best_effort();
        _pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("timing_test", pub_qos_settings);

        _timer = create_wall_timer(5ms, std::bind(&pub::timer_udpate, this));
    }
private:
    void timer_udpate(void)
    {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        _pub->publish(msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pub>());
    rclcpp::shutdown();
    return 0;
}
