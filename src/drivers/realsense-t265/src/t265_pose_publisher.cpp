#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
using namespace std::chrono_literals;


class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher()
    : Node("t265_pose_pub")
    {
        this->declare_parameter("minimum_confidence", 3);
        this->declare_parameter("max_pub_rate", 50);
        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("t265/pose", 10);
        twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("t265/twist", 10);
        auto poll_loop_thd = std::thread(std::bind(&PosePublisher::poll_loop_thd_fn, this));
        poll_loop_thd.detach();
    }

private:
    void poll_loop_thd_fn()
    {
        while (true) {
            try {
                poll_loop();
            }
            catch (const rs2::error & e)
            {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << std::endl;
            }
        }
    }

    void poll_loop()
    {
        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;
        // Add pose stream
        cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        // Start pipeline with chosen configuration
        pipe.start(cfg);

        // Main loop
        while (true) {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            // Get a frame from the pose stream
            auto f = frames.first_or_default(RS2_STREAM_POSE);
            // Cast the frame to pose_frame and get its data
            auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
            auto frame_time = now(); // TODO
            if ((frame_time - last_pub).seconds() < 1.0/get_parameter("max_pub_rate").as_int()) {
                continue;
            }
            if (pose_data.tracker_confidence >= this->get_parameter("minimum_confidence").as_int()) {
                auto pose_msg = geometry_msgs::msg::PoseStamped();
                pose_msg.header.stamp = frame_time;
                pose_msg.header.frame_id = "t265_map";
                pose_msg.pose.position.x = pose_data.translation.x;
                pose_msg.pose.position.y = pose_data.translation.y;
                pose_msg.pose.position.z = pose_data.translation.z;
                pose_msg.pose.orientation.w = pose_data.rotation.w;
                pose_msg.pose.orientation.x = pose_data.rotation.x;
                pose_msg.pose.orientation.y = pose_data.rotation.y;
                pose_msg.pose.orientation.z = pose_data.rotation.z;
                pose_pub->publish(pose_msg);
                auto twist_msg = geometry_msgs::msg::TwistStamped();
                twist_msg.header.stamp = frame_time;
                twist_msg.header.frame_id = "t265_map";
                twist_msg.twist.linear.x = pose_data.velocity.x;
                twist_msg.twist.linear.y = pose_data.velocity.y;
                twist_msg.twist.linear.z = pose_data.velocity.z;
                twist_msg.twist.angular.x = pose_data.angular_velocity.x;
                twist_msg.twist.angular.y = pose_data.angular_velocity.y;
                twist_msg.twist.angular.z = pose_data.angular_velocity.z;
                twist_pub->publish(twist_msg);
                last_pub = frame_time;
            }
            // // Print the x, y, z values of the translation, relative to initial position
            // std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
            //     pose_data.translation.y << " " << pose_data.translation.z << " (meters)";

        }
    }

    rclcpp::Time last_pub{{0, RCL_ROS_TIME}};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}