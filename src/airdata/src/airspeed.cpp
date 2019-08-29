#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// todo: these two lines are needed because of the Node being created with make shared
// http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <Eigen/Dense>
#include <string>

#define AIR_DENSITY 1.225f

class Airspeed : public rclcpp::Node
{
public:
    Airspeed()
    : Node("Airspeed"),
        aoa_pressure_update((int64_t)0, RCL_ROS_TIME),
        aos_pressure_update((int64_t)0, RCL_ROS_TIME)
    {
        C_aoa = 1;
        C_aos = 1;

        dynamic_pressure_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "/airdata/dynamic_pressure", 10, std::bind(&Airspeed::dynamic_pressure_cb, this, _1));
        aoa_pressure_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "/airdata/aoa_pressure", 10, std::bind(&Airspeed::aoa_pressure_cb, this, _1));
        aos_pressure_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "/airdata/aos_pressure", 10, std::bind(&Airspeed::aos_pressure_cb, this, _1));

        airspeed_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/airdata/airspeed", 10);
    }

private:
    double dynamic_pressure_to_speed(double pressure)
    {
        if (pressure < 0) {
            return 0;
        }
        return sqrt(2*pressure / AIR_DENSITY);
    }

    void dynamic_pressure_cb(sensor_msgs::msg::FluidPressure::SharedPtr msg)
    {
        dynamic_pressure = msg->fluid_pressure;
        rclcpp::Time now = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "dynamic_pressure %f", dynamic_pressure);
        if (fabs((now - aoa_pressure_update).seconds()) < 0.1 && fabs((now - aos_pressure_update).seconds()) < 0.1) {
            // 3D airspeed
            double vx = dynamic_pressure_to_speed(dynamic_pressure);
            double vy = 0;
            double vz = 0;
            if (dynamic_pressure > 0.1) {
                vz = vx * C_aoa * aoa_pressure/dynamic_pressure;
                vy = vx * C_aos * aos_pressure/dynamic_pressure;
            }

            auto airspeed_msg = geometry_msgs::msg::Vector3Stamped();
            airspeed_msg.header.frame_id = "body";
            airspeed_msg.header.stamp = now;
            airspeed_msg.vector.x = vx;
            airspeed_msg.vector.y = vy;
            airspeed_msg.vector.z = vz;
            airspeed_pub->publish(airspeed_msg);
        } else {
            // scalar airspeed
            RCLCPP_WARN(this->get_logger(), "3d airspeed not available");
        }
    }

    void aoa_pressure_cb(sensor_msgs::msg::FluidPressure::SharedPtr msg)
    {
        aoa_pressure = msg->fluid_pressure;
        aoa_pressure_update = msg->header.stamp;
    }

    void aos_pressure_cb(sensor_msgs::msg::FluidPressure::SharedPtr msg)
    {
        aos_pressure = msg->fluid_pressure;
        aos_pressure_update = msg->header.stamp;
    }

    double dynamic_pressure=0;
    double aoa_pressure=0;
    rclcpp::Time aoa_pressure_update;
    double aos_pressure=0;
    rclcpp::Time aos_pressure_update;

    double C_aoa;
    double C_aos;

    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr dynamic_pressure_sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr aoa_pressure_sub;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr aos_pressure_sub;
    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr airspeed_pub;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Airspeed>());
    rclcpp::shutdown();
    return 0;
}
