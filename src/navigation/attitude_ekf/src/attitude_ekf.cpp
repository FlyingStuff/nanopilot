#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
// todo: these two lines are needed because of the Node being created with make shared
// http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
#include <iostream>
#include <string>
// #include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include "ekf.hpp"

Eigen::Matrix3d cross_product_matrix(Eigen::Vector3d v)
{
    Eigen::Matrix3d vx;
    vx <<     0, -v[2],  v[1],
           v[2],     0, -v[0],
          -v[1],  v[0],     0;
    return vx;
}


double deg_to_rad(double deg)
{
    return deg*M_PI/180;
}

double sq(double x)
{
    return x*x;
}

class AttitudeEKF : public rclcpp::Node
{
public:
    AttitudeEKF()
    : Node("AttitudeEKF"),
    body_frame("body"),
    inertial_frame("NED")
    {
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", std::bind(&AttitudeEKF::imu_cb, this, _1));
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vrpn_client_node/quad/pose", std::bind(&AttitudeEKF::pose_cb, this, _1));


        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose");

        x.setZero();
        P0.setZero();
        P0.diagonal() <<
            sq(deg_to_rad(180)), sq(deg_to_rad(180)), sq(deg_to_rad(180)), // angle covariance in body [(rad)^2]
            sq(deg_to_rad(2)), sq(deg_to_rad(2)), sq(deg_to_rad(2)); // gyro bias covariance [(rad/s)^2]
        Q.setZero();
        double gyro_noise = 0.001; // gyro noise density [rad/s/sqrt(Hz)]
        double gyro_random_walk = 0.0001; // gyro bias random walk [rad/s^2/sqrt(Hz)]
        Q.diagonal() <<
            sq(gyro_noise), sq(gyro_noise), sq(gyro_noise),
            sq(gyro_random_walk), sq(gyro_random_walk), sq(gyro_random_walk);
        std::cout << Q << std::endl;
        R_ACC.setZero();
        R_ACC.diagonal() <<
            sq(0.1), sq(0.1), sq(0.1); // covariance of a unit vector
        R_QUAT.setZero();
        R_QUAT.diagonal() <<
            sq(deg_to_rad(10)), sq(deg_to_rad(10)), sq(deg_to_rad(10)); // angle covariance in body [(rad)^2]
        R_MAG.setZero();
        this->reset();
    }


    void reset()
    {
        this->x.setZero();
        this->attitude_reference_B_to_I = Eigen::Quaternion<double>::Identity();
        this->P = this->P0;
    }

private:

    void time_update(Eigen::Quaternion<double> &current_b_to_prev_b, double delta_t)
    {

        Eigen::Vector3d omega_gyro = 2*current_b_to_prev_b.vec()/current_b_to_prev_b.w()/delta_t;

        Eigen::Vector3d gyro_bias = x.block<3,1>(3, 0);
        Eigen::Vector3d omega_b = omega_gyro - gyro_bias;
        // RCLCPP_INFO(this->get_logger(), "gyro bias '%f %f %f'", omega_b[0], omega_b[1], omega_b[2]);

        // update reference
        Eigen::Quaternion<double> body_rotation(1, delta_t*omega_b[0]/2, delta_t*omega_b[1]/2, delta_t*omega_b[2]/2);
        this->attitude_reference_B_to_I = this->attitude_reference_B_to_I * body_rotation;

        // update states
        //   no change

        // update covariance
        Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;
        F.setZero();
        F.block<3,3>(0, 0) = -1*cross_product_matrix(omega_b); // top left block
        F.block<3,3>(0, 3) = -1*Eigen::Matrix<double, 3, 3>::Identity(); // top right block
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

    void acc_measurement_update(Eigen::Vector3d &acc)
    {
        Eigen::Vector3d acc_dir = acc.normalized();
        Eigen::Vector3d acc_dir_inertial_expected(0, 0, -1); // accelerating up in NED
        Eigen::Vector3d acc_dir_body_expected;
        acc_dir_body_expected = attitude_reference_B_to_I.conjugate()*acc_dir_inertial_expected;
        // RCLCPP_INFO(this->get_logger(), "acc exp '%f %f %f'", acc_dir_body_expected[0], acc_dir_body_expected[1], acc_dir_body_expected[2]);
        // RCLCPP_INFO(this->get_logger(), "acc actual '%f %f %f'", acc_dir[0], acc_dir[1], acc_dir[2]);

        Eigen::Matrix<double, ACC_MEASURE_DIM, STATE_DIM> H;
        H.setZero();
        H.block<3,3>(0, 0) = cross_product_matrix(acc_dir_body_expected);

        Eigen::Matrix<double, ACC_MEASURE_DIM, 1> y;
        Eigen::Matrix<double, ACC_MEASURE_DIM, ACC_MEASURE_DIM> S;
        Eigen::Matrix<double, STATE_DIM, ACC_MEASURE_DIM> K;

        y = acc_dir - acc_dir_body_expected;
        S = H * P * H.transpose() + R_ACC;
        K = P * H.transpose() * S.inverse(); // todo use pseudo inverse to be safe

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

    void quaternion_measurement_update(Eigen::Quaterniond &measured_B_to_I)
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
        K = P * H.transpose() * S.inverse(); // todo use pseudo inverse to be safe

        auto &I = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
        P = (I - K * H) * P;
        x = x + K * y;
        attitude_error_to_reference_transfer();
    }

    void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // todo use tf2
        Eigen::Quaterniond imu_to_body(Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitZ())
                                        * Eigen::AngleAxisd(M_PI,  Eigen::Vector3d::UnitY()));

        if (prev_imu_msg) {
            rclcpp::Duration dt = rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_imu_msg->header.stamp);
            // RCLCPP_INFO(this->get_logger(), "*****************");
            // RCLCPP_INFO(this->get_logger(), "imu delta t %f", dt.seconds());
            // RCLCPP_INFO(this->get_logger(), "imu omega: '%f %f %f'", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            Eigen::Quaternion<double> current_imu_to_I(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            Eigen::Quaternion<double> prev_imu_to_I(prev_imu_msg->orientation.w, prev_imu_msg->orientation.x, prev_imu_msg->orientation.y, prev_imu_msg->orientation.z);
            Eigen::Quaternion<double> current_b_to_I = current_imu_to_I*imu_to_body.conjugate();
            Eigen::Quaternion<double> prev_b_to_I = prev_imu_to_I*imu_to_body.conjugate();
            auto current_b_to_prev_b = prev_b_to_I.conjugate()*current_b_to_I; // transform from current body to previous body frame
            // RCLCPP_INFO(this->get_logger(), "imu dq '%f %f %f %f'", current_b_to_prev_b.w(), current_b_to_prev_b.x(), current_b_to_prev_b.y(), current_b_to_prev_b.z());
            // RCLCPP_INFO(this->get_logger(), "imu omega(dq,dt) '%f %f %f'", current_b_to_prev_b.x()*2/dt.seconds(), current_b_to_prev_b.y()*2/dt.seconds(), current_b_to_prev_b.z()*2/dt.seconds());

            time_update(current_b_to_prev_b, dt.seconds());

            // RCLCPP_INFO(this->get_logger(), "ref att '%f %f %f %f'",
            //     this->attitude_reference_B_to_I.w(),
            //     this->attitude_reference_B_to_I.x(),
            //     this->attitude_reference_B_to_I.y(),
            //     this->attitude_reference_B_to_I.z());

            Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            auto acc_body = imu_to_body*acc;
            acc_measurement_update(acc_body);

            // RCLCPP_INFO(this->get_logger(), "ref att '%f %f %f %f'",
            //     this->attitude_reference_B_to_I.w(),
            //     this->attitude_reference_B_to_I.x(),
            //     this->attitude_reference_B_to_I.y(),
            //     this->attitude_reference_B_to_I.z());

            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.frame_id = inertial_frame;
            pose_msg.header.stamp = rclcpp::Time(msg->header.stamp);
            pose_msg.pose.orientation.w = attitude_reference_B_to_I.w();
            pose_msg.pose.orientation.x = attitude_reference_B_to_I.x();
            pose_msg.pose.orientation.y = attitude_reference_B_to_I.y();
            pose_msg.pose.orientation.z = attitude_reference_B_to_I.z();
            pose_pub->publish(pose_msg);
        }
        prev_imu_msg = msg;
    }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        auto measured_B_to_I = Eigen::Quaterniond(msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        quaternion_measurement_update(measured_B_to_I);
    }

    std::string body_frame;
    std::string inertial_frame;

    static const int STATE_DIM=6; // attitude err [3], gyro bias [3]
    static const int MAG_MEASURE_DIM=3;
    static const int ACC_MEASURE_DIM=3;
    static const int QUATERNION_MEASURE_DIM=3;

    // States
    Eigen::Quaternion<double> attitude_reference_B_to_I; // body to inerital transform
    Eigen::Matrix<double, STATE_DIM, 1> x;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P;
    // Parameters
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;
    Eigen::Matrix<double, ACC_MEASURE_DIM, ACC_MEASURE_DIM> R_ACC;
    Eigen::Matrix<double, QUATERNION_MEASURE_DIM, QUATERNION_MEASURE_DIM> R_QUAT;
    Eigen::Matrix<double, MAG_MEASURE_DIM, MAG_MEASURE_DIM> R_MAG;

    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    sensor_msgs::msg::Imu::SharedPtr prev_imu_msg;
    // ROS Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

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
