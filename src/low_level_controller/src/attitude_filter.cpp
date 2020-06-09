#include "attitude_filter.hpp"
#include "control_loop.hpp"
#include <Eigen/Dense>
#include "sensors.hpp"
#include <math.h>
#include <algorithm>

msgbus::Topic<attitude_filter_output_t> attitude_filter_output_topic;
msgbus::Topic<external_attitude_reference_t> attitude_reference_topic;

static parameter_t R_board_to_body_param;
static Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_board_to_body;
static Eigen::Quaternionf q_body_to_board;
static auto imu_sub = msgbus::subscribe(imu);
static auto att_ref_sub = msgbus::subscribe(attitude_reference_topic);


/** Find the quaternion that rotates v1 -> v2 and is the shortest rotation
 */
Eigen::Quaternionf shortest_rotation_quaternion(
    const Eigen::Vector3f &v1,
    const Eigen::Vector3f &v2)
{
    auto v1n = v1.normalized();
    auto v2n = v2.normalized();
    auto c = (v1n + v2n).normalized(); // TODO handle case where norm is zero
    auto qv = v1n.cross(c);
    Eigen::Quaternionf q(v1n.dot(c), qv[0], qv[1], qv[2]);
    return q;
}
// TODO move this function to a shared place



class AttitudeFilter
{
    Eigen::Quaternionf attitude_estimate{Eigen::Quaternionf(1, 0, 0, 0)};
    Eigen::Quaternionf prev_accumulated_angle{Eigen::Quaternionf(1, 0, 0, 0)};
    Eigen::Vector3f angular_rate{Eigen::Vector3f(0, 0, 0)};
    timestamp_t attitude_estimate_time{0};
    timestamp_t last_attitude_reference_update{0};
    timestamp_t last_gravity_measurement{0};

public:
    // parameters
    float gravity_align_time_cst{3.0f}; // [s]
    float attitude_reference_align_time_cst{3.0f}; // [s]
    float reference_timeout{5.0f}; // [s]
    float reference_acceptance_time_delta{0.1f}; // [s]

    const Eigen::Quaternionf &get_attitude() { return attitude_estimate; }
    const Eigen::Vector3f &get_angular_rate() { return angular_rate; }
    timestamp_t get_timestamp() { return attitude_estimate_time; }

    void time_update(const Eigen::Quaternionf &accumulated_angle, const Eigen::Vector3f &angular_rate, timestamp_t timestamp)
    {
        auto delta = prev_accumulated_angle.conjugate() * accumulated_angle;
        if (attitude_estimate_time != 0) {
            attitude_estimate = attitude_estimate * delta;
        }
        attitude_estimate_time = timestamp;
        prev_accumulated_angle = accumulated_angle;
        this->angular_rate = angular_rate;
    }

    void measure_gravity(const Eigen::Vector3f &acceleration, timestamp_t timestamp)
    {
        float measurement_period = std::max(0.0f, timestamp_duration(last_gravity_measurement, timestamp));
        last_gravity_measurement = timestamp;

        if (reference_valid()) {
            return; // no gravity updates if we have an attitude reference
        }

        // no time correction is made, acceleration is synchronized with gyro time update

        float acc_norm = acceleration.norm();
        if (acc_norm < 2.0f || acc_norm > 20.0f) {
            return; // very low/high acceleration, don't use it
        }
        auto acc_dir_b = acceleration/acc_norm;
        auto acc_dir_I = attitude_estimate._transformVector(acc_dir_b);
        const Eigen::Vector3f acc_dir_true_I(0, 0, -1);
        auto att_err_I = shortest_rotation_quaternion(acc_dir_I, acc_dir_true_I); // estimate inertial to true interial transform

        Eigen::Vector3f g = att_err_I.vec() / att_err_I.w(); // Rodriques parameter vector

        float eff_gravity_align_time_cst = gravity_align_time_cst;
        if (acc_norm > 12.0f || acc_norm < 8.0f) { // more movement -> increase time constant
            eff_gravity_align_time_cst = 5*gravity_align_time_cst;
        }
        float alpha = measurement_period / (eff_gravity_align_time_cst + measurement_period);
        auto g_adj = alpha * g;

        Eigen::Quaternionf q_adj(1, g_adj[0], g_adj[1], g_adj[2]); // rodrigues->quaternion, no need to normalize since we normalize at the end
        // q_adj.normalize();
        attitude_estimate = q_adj * attitude_estimate;
        attitude_estimate.normalize();
    }

    void measure_attitude_reference(const Eigen::Quaternionf &reference, timestamp_t ref_timestamp)
    {
        float ref_age = timestamp_duration(ref_timestamp, attitude_estimate_time);
        if (ref_age > reference_acceptance_time_delta || ref_age < -reference_acceptance_time_delta) {
            return; // reference too old (or too far in the future)
        }

        float measurement_period = std::max(0.0f, timestamp_duration(last_attitude_reference_update, ref_timestamp));
        last_attitude_reference_update = ref_timestamp;

        // translate attitude reference to attitude_estimate_time using angular rate
        auto qv = 0.5f*ref_age*angular_rate;
        auto reference_now = reference * Eigen::Quaternionf(1, qv[0], qv[1], qv[2]); // good for small angles (=small delta_t)

        auto attitude_error_body = attitude_estimate.conjugate() * reference_now; // reference_body -> estimate_body transform
        Eigen::Vector3f g = attitude_error_body.vec() / attitude_error_body.w(); // Rodriques parameter vector

        float alpha = measurement_period / (attitude_reference_align_time_cst + measurement_period);
        alpha = std::min(0.2f, alpha); // limit alpha in case the measurement period is too large
        auto g_adj = alpha * g;

        Eigen::Quaternionf q_adj(1, g_adj[0], g_adj[1], g_adj[2]); // rodrigues->quaternion, no need to normalize since we normalize at the end
        // q_adj.normalize();
        attitude_estimate = attitude_estimate * q_adj;
        attitude_estimate.normalize();
    }

    bool reference_valid()
    {
        float delta_t = timestamp_duration(last_attitude_reference_update, attitude_estimate_time);
        return delta_t < reference_timeout && last_attitude_reference_update != 0;
    }
};


static AttitudeFilter attitude_filter;

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

    if (imu_sub.has_update()) {
        imu_sample_t imu = imu_sub.get_value();
        // transform to body frame
        Eigen::Map<Eigen::Vector3f> rate_measured_board(imu.angular_rate);
        Eigen::Map<Eigen::Vector3f> acc_measured_board(imu.linear_acceleration);
        Eigen::Quaternionf q_board_to_fixed(
            imu.accumulated_angle.w,
            imu.accumulated_angle.x,
            imu.accumulated_angle.y,
            imu.accumulated_angle.z);
        Eigen::Quaternionf accumulated_angle_b = q_board_to_fixed * q_body_to_board;
        Eigen::Vector3f omega_b = R_board_to_body * rate_measured_board;
        Eigen::Vector3f acc_b = R_board_to_body * acc_measured_board;

        attitude_filter.time_update(accumulated_angle_b, omega_b, imu.timestamp);
        attitude_filter.measure_gravity(acc_b, imu.timestamp);
    }

    if (att_ref_sub.has_update()) {
        external_attitude_reference_t att_ref = att_ref_sub.get_value();
        Eigen::Quaternionf q_ref(
            att_ref.attitude_reference.w,
            att_ref.attitude_reference.x,
            att_ref.attitude_reference.y,
            att_ref.attitude_reference.z);
        if (isfinite(att_ref.attitude_reference.w)
            && isfinite(att_ref.attitude_reference.x)
            && isfinite(att_ref.attitude_reference.y)
            && isfinite(att_ref.attitude_reference.z)
            && q_ref.squaredNorm() < 1.001f && q_ref.squaredNorm() > 0.999f) {
            attitude_filter.measure_attitude_reference(q_ref, att_ref.timestamp);
        }

    }

    attitude_filter_output_t attitude_filter_out;
    attitude_filter_out.reference_valid = attitude_filter.reference_valid();
    auto omega = attitude_filter.get_angular_rate();
    attitude_filter_out.angular_rate[0] = omega[0];
    attitude_filter_out.angular_rate[1] = omega[1];
    attitude_filter_out.angular_rate[2] = omega[2];
    auto att = attitude_filter.get_attitude();
    attitude_filter_out.attitude.w = att.w();
    attitude_filter_out.attitude.x = att.x();
    attitude_filter_out.attitude.y = att.y();
    attitude_filter_out.attitude.z = att.z();
    attitude_filter_out.timestamp = attitude_filter.get_timestamp();
    attitude_filter_output_topic.publish(attitude_filter_out);
}

