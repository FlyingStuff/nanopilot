#include "attitude_filter.hpp"
#include "control_loop.hpp"
#include <Eigen/Dense>
#include "sensors.hpp"

msgbus::Topic<attitude_filter_output_t> attitude_filter_output_topic;
msgbus::Topic<external_attitude_reference_t> attitude_reference_topic;

static parameter_t R_board_to_body_param;
static Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_board_to_body;
static Eigen::Quaternionf q_body_to_board;
static auto imu_sub = msgbus::subscribe(imu);

void attitude_filter_init()
{
    static float R_board_to_body_buf[9] = {1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
    parameter_vector_declare_with_default(&R_board_to_body_param, &control_ns, "R_board_to_body", R_board_to_body_buf, 9);
}


void attitude_filter_update()
{
    if (parameter_changed(&R_board_to_body_param)) {
        parameter_vector_get(&R_board_to_body_param, R_board_to_body.data());
        q_body_to_board = Eigen::Quaternionf(R_board_to_body).conjugate();
    }
    // TODO handle non published imu correctly
    attitude_filter_output_t attitude_filter_out;

    bool imu_updated = imu_sub.has_update();
    if (imu_updated) {
        imu_sample_t imu = imu_sub.get_value();
        // transform to body frame
        Eigen::Map<Eigen::Vector3f> rate_measured_board(imu.angular_rate);
        Eigen::Quaternionf q_board_to_fixed(
            imu.accumulated_angle.w,
            imu.accumulated_angle.x,
            imu.accumulated_angle.y,
            imu.accumulated_angle.z);
        Eigen::Quaternionf accumulated_angle_b = q_board_to_fixed * q_body_to_board;
        Eigen::Vector3f omega_b = R_board_to_body * rate_measured_board;


        // time update
        attitude_filter_out.reference_valid = false;
        attitude_filter_out.angular_rate[0] = omega_b[0];
        attitude_filter_out.angular_rate[1] = omega_b[1];
        attitude_filter_out.angular_rate[2] = omega_b[2];
        attitude_filter_out.attitude.w = accumulated_angle_b.w(); // TODO fix this
        attitude_filter_out.attitude.x = accumulated_angle_b.x();
        attitude_filter_out.attitude.y = accumulated_angle_b.y();
        attitude_filter_out.attitude.z = accumulated_angle_b.z();
        attitude_filter_out.timestamp = imu.timestamp;
    }
    // if external reference
    // else if external ref timeout
    // if imu_updated: run acc correction
    if (imu_updated) {
        attitude_filter_output_topic.publish(attitude_filter_out);
    }
}

