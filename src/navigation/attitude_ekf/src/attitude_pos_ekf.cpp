#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// todo: these two lines are needed because of the Node being created with make shared
// http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_eigen/tf2_eigen.h>
using std::placeholders::_1;
#include <iostream>
#include <string>
#include <chrono>
using namespace std::chrono_literals;


#include <Eigen/Dense>

static const double GRAVITY = 9.81;

Eigen::Matrix3d cross_product_matrix(Eigen::Vector3d v)
{
    Eigen::Matrix3d vx;
    vx <<     0, -v[2],  v[1],
           v[2],     0, -v[0],
          -v[1],  v[0],     0;
    return vx;
}


static double deg_to_rad(double deg)
{
    return deg*M_PI/180;
}

static double sq(double x)
{
    return x*x;
}

class AttitudeEKF : public rclcpp::Node
{
public:
    AttitudeEKF()
    : Node("AttitudePosEKF"),
    body_frame("body"),
    inertial_frame("NED"),
    measurement_pose_frame("t265")
    {
        rclcpp::QoS pub_qos_settings(10);
        rclcpp::QoS sub_qos_settings(1);
        sub_qos_settings.best_effort();

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", sub_qos_settings, std::bind(&AttitudeEKF::imu_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/t265/pose", sub_qos_settings, std::bind(&AttitudeEKF::pose_cb, this, _1));

        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", pub_qos_settings);
        twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", pub_qos_settings);
        accel_pub = this->create_publisher<geometry_msgs::msg::AccelStamped>("accel", pub_qos_settings);

        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        tf_buffer->setCreateTimerInterface(timer_interface);
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        x.setZero();
        P0.setZero();
        P0.diagonal() <<
            sq(deg_to_rad(180)), sq(deg_to_rad(180)), sq(deg_to_rad(180)), // angle covariance in body [(rad)^2]
            sq(deg_to_rad(2)), sq(deg_to_rad(2)), sq(deg_to_rad(2)), // gyro bias covariance [(rad/s)^2]
            sq(100), sq(100), sq(100), // position covariance [(m)^2]
            sq(10), sq(10), sq(10); // velocity covariance [(m/s)^2]
        Q.setZero();
        double gyro_noise = deg_to_rad(0.004); // gyro noise density [rad/s/sqrt(Hz)]
        // double gyro_random_walk = deg_to_rad(0.0002); // gyro bias random walk [rad/s^2/sqrt(Hz)]
        double gyro_random_walk = deg_to_rad(0.02); // gyro bias random walk [rad/s^2/sqrt(Hz)]
        double acc_noise = 0.2; // [m/s2/sqrt(Hz)]
        Q.diagonal() <<
            sq(gyro_noise), sq(gyro_noise), sq(gyro_noise),
            sq(gyro_random_walk), sq(gyro_random_walk), sq(gyro_random_walk),
            0, 0, 0, // no additive velocity noise
            sq(acc_noise), sq(acc_noise), sq(acc_noise); // accelerometer noise
        std::cout << Q << std::endl;
        R_POS.setZero();
        R_POS.diagonal() <<
            sq(0.1), sq(0.1), sq(0.1); // covariance of a position measurement [(m)^2]
        R_QUAT.setZero();
        R_QUAT.diagonal() <<
            sq(deg_to_rad(20)), sq(deg_to_rad(20)), sq(deg_to_rad(5)); // angle covariance in body [(rad)^2]
        this->reset();
    }


    void reset()
    {
        this->x.setZero();
        this->attitude_reference_B_to_I = Eigen::Quaternion<double>::Identity();
        this->P = this->P0;
    }

private:
    void publish_estimate(rclcpp::Time now)
    {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.frame_id = inertial_frame;
        pose_msg.header.stamp = now;
        pose_msg.pose.orientation.w = attitude_reference_B_to_I.w();
        pose_msg.pose.orientation.x = attitude_reference_B_to_I.x();
        pose_msg.pose.orientation.y = attitude_reference_B_to_I.y();
        pose_msg.pose.orientation.z = attitude_reference_B_to_I.z();
        pose_msg.pose.position.x = x[6];
        pose_msg.pose.position.y = x[7];
        pose_msg.pose.position.z = x[8];
        pose_pub->publish(pose_msg);

        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.frame_id = inertial_frame;
        twist_msg.header.stamp = now;
        twist_msg.twist.angular.x = body_angular_rate[0];
        twist_msg.twist.angular.y = body_angular_rate[1];
        twist_msg.twist.angular.z = body_angular_rate[2];
        twist_msg.twist.linear.x = x[9];
        twist_msg.twist.linear.y = x[10];
        twist_msg.twist.linear.z = x[11];
        twist_pub->publish(twist_msg);
    }

    void time_update(Eigen::Quaternion<double> &current_b_to_prev_b, double delta_t, Eigen::Vector3d &acc)
    {

        Eigen::Vector3d omega_gyro = 2*current_b_to_prev_b.vec()/current_b_to_prev_b.w()/delta_t;

        Eigen::Vector3d gyro_bias = x.block<3,1>(3, 0);
        Eigen::Vector3d omega_b = omega_gyro - gyro_bias;
        body_angular_rate = omega_b;

        // update reference
        Eigen::Quaternion<double> body_rotation(1, delta_t*omega_b[0]/2, delta_t*omega_b[1]/2, delta_t*omega_b[2]/2);
        this->attitude_reference_B_to_I = this->attitude_reference_B_to_I * body_rotation;

        Eigen::Matrix<double, 3, 3> R_b_to_I = attitude_reference_B_to_I.toRotationMatrix();
        // update states
        x.block<3,1>(6, 0) += x.block<3,1>(9, 0) * delta_t; // p += v*dt
        Eigen::Vector3d acc_I = R_b_to_I * acc + Eigen::Vector3d(0, 0, GRAVITY);
        x.block<3,1>(9, 0) += acc_I * delta_t;

        // update covariance
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;
        F.setZero();
        F.block<3,3>(0, 0) = -1*cross_product_matrix(omega_b); // da_dot/da
        F.block<3,3>(0, 3) = -1*Eigen::Matrix<double, 3, 3>::Identity(); // da_dot/db
        F.block<3,3>(6, 9) = Eigen::Matrix<double, 3, 3>::Identity(); // dp_dot/dv
        F.block<3,3>(9, 0) = -1*R_b_to_I * cross_product_matrix(acc); // dv_dot/da

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> Phi;
        Phi.setIdentity();
        Phi += F*delta_t;
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> Qs = Phi *Q;
        P = Phi * P * Phi.transpose() + Qs;
    }

    void attitude_error_to_reference_transfer()
    {
        Eigen::Quaternion<double> attitude_error_q;
        double a_norm_sq = x.block<3,1>(0,0).squaredNorm();
        attitude_error_q.w() = 2/sqrt(4 + a_norm_sq);
        attitude_error_q.x() = x(0,0)/sqrt(4 + a_norm_sq);
        attitude_error_q.y() = x(1,0)/sqrt(4 + a_norm_sq);
        attitude_error_q.z() = x(2,0)/sqrt(4 + a_norm_sq);
        this->attitude_reference_B_to_I = this->attitude_reference_B_to_I * attitude_error_q;
        this->x.block<3,1>(0, 0).setZero(); // reset error estimate
        this->attitude_reference_B_to_I.normalize();
    }

    void pos_measurement_update(const Eigen::Vector3d &pos)
    {
        Eigen::Matrix<double, POS_MEASURE_DIM, STATE_DIM> H;
        H.setZero();
        H.block<3,3>(0, 6).setIdentity();

        Eigen::Matrix<double, POS_MEASURE_DIM, 1> y;
        Eigen::Matrix<double, POS_MEASURE_DIM, POS_MEASURE_DIM> S;
        Eigen::Matrix<double, STATE_DIM, POS_MEASURE_DIM> K;

        y = pos - x.block<3,1>(6, 0);
        S = H * P * H.transpose() + R_POS;
        K = P * H.transpose() * S.inverse();

        Eigen::Matrix<double, STATE_DIM, STATE_DIM> I;
        I.setIdentity();
        P = (I - K * H) * P;
        x = x + K * y;

        attitude_error_to_reference_transfer();
    }

    void mag_measurement_update()
    {
        attitude_error_to_reference_transfer();
    }

    void quaternion_measurement_update(const Eigen::Quaterniond &measured_B_to_I)
    {
        Eigen::Matrix<double, QUATERNION_MEASURE_DIM, STATE_DIM> H;
        H.setZero();
        H.block<3,3>(0, 0).setIdentity();

        Eigen::Matrix<double, QUATERNION_MEASURE_DIM, 1> y;
        Eigen::Matrix<double, QUATERNION_MEASURE_DIM, QUATERNION_MEASURE_DIM> S;
        Eigen::Matrix<double, STATE_DIM, QUATERNION_MEASURE_DIM> K;

        auto q_error = attitude_reference_B_to_I.conjugate() * measured_B_to_I;
        Eigen::Vector3d a_measured = 2/q_error.w()*q_error.vec();

        y = a_measured; // a_expected is zero
        S = H * P * H.transpose() + R_QUAT;
        K = P * H.transpose() * S.inverse();

        auto &I = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        P = (I - K * H) * P;
        x = x + K * y;
        attitude_error_to_reference_transfer();
    }

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Eigen::Quaterniond imu_to_body;
        try {
            auto imu_to_body_tf = tf_buffer->lookupTransform(
                body_frame /*target*/, msg->header.frame_id /*source*/, tf2::TimePointZero/*time 0 => latest*/);
            imu_to_body = Eigen::Quaterniond(tf2::transformToEigen(imu_to_body_tf).rotation());
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN(get_logger(), "TF2 transform failed: %s", e.what());
            return;
        }

        if (prev_imu_msg) {
            auto current_time = rclcpp::Time(msg->header.stamp);
            rclcpp::Duration dt = current_time - rclcpp::Time(prev_imu_msg->header.stamp);
            Eigen::Quaternion<double> current_imu_to_I(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            Eigen::Quaternion<double> prev_imu_to_I(prev_imu_msg->orientation.w, prev_imu_msg->orientation.x, prev_imu_msg->orientation.y, prev_imu_msg->orientation.z);
            Eigen::Quaternion<double> current_b_to_I = current_imu_to_I*imu_to_body.conjugate();
            Eigen::Quaternion<double> prev_b_to_I = prev_imu_to_I*imu_to_body.conjugate();
            auto current_b_to_prev_b = prev_b_to_I.conjugate()*current_b_to_I; // transform from current body to previous body frame

            Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            auto acc_body = imu_to_body*acc;

            time_update(current_b_to_prev_b, dt.seconds(), acc_body);
            time = current_time;
            publish_estimate(current_time);

            if (dt > rclcpp::Duration(50ms)) {
                RCLCPP_WARN(this->get_logger(), "imu dt high (%f s)", dt.seconds());
            }

            // Eigen::Vector3d tmp(0, 0, 0);
            // pos_measurement_update(tmp);
        }
        prev_imu_msg = msg;
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        auto msg_age = time - rclcpp::Time(msg->header.stamp);
        if (msg_age > rclcpp::Duration(50ms)) {
            RCLCPP_WARN(this->get_logger(), "msg time too far from current time (%f s)", msg_age.seconds());
            return;
        }

        try {
            auto pose_ref_to_inertial_tf = tf_buffer->lookupTransform(
                inertial_frame /*target*/, msg->header.frame_id /*source*/, tf2::TimePointZero/*time 0 => latest*/);
            Eigen::Affine3d pose_ref_to_inertial = tf2::transformToEigen(pose_ref_to_inertial_tf);

            auto body_to_measured_pose_tf = tf_buffer->lookupTransform(
                measurement_pose_frame /*target*/, body_frame /*source*/, tf2::TimePointZero/*time 0 => latest*/);
            Eigen::Affine3d body_to_measured_pose = tf2::transformToEigen(body_to_measured_pose_tf);

            Eigen::Affine3d measured_pose_to_pose_ref;
            tf2::fromMsg(msg->pose, measured_pose_to_pose_ref);

            // chain all transforms
            Eigen::Affine3d measured_body_to_inertial = pose_ref_to_inertial * measured_pose_to_pose_ref * body_to_measured_pose;
            Eigen::Vector3d pos_I = measured_body_to_inertial.translation();
            Eigen::Quaterniond measured_B_to_I(measured_body_to_inertial.rotation());

            pos_measurement_update(pos_I);
            RCLCPP_INFO(this->get_logger(), "pose update '%f %f %f'", pos_I[0], pos_I[1], pos_I[2]);

            quaternion_measurement_update(measured_B_to_I);

        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN(get_logger(), "TF2 transform failed: %s", e.what());
        }
    }

    std::string body_frame;
    std::string inertial_frame;
    std::string measurement_pose_frame;

    static const int STATE_DIM=12; // attitude err [3], gyro bias [3], position [3], velocity [3]
    static const int QUATERNION_MEASURE_DIM=3;
    static const int POS_MEASURE_DIM=3;

    // States
    Eigen::Quaternion<double> attitude_reference_B_to_I; // body to inerital transform
    Eigen::Matrix<double, STATE_DIM, 1> x;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P;
    rclcpp::Time time{{0, RCL_ROS_TIME}};
    // Parameters
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;
    Eigen::Matrix<double, QUATERNION_MEASURE_DIM, QUATERNION_MEASURE_DIM> R_QUAT;
    Eigen::Matrix<double, POS_MEASURE_DIM, POS_MEASURE_DIM> R_POS;

    Eigen::Vector3d body_angular_rate;

    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    sensor_msgs::msg::Imu::SharedPtr prev_imu_msg;
    // tf2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudeEKF>());
    rclcpp::shutdown();
    return 0;
}
