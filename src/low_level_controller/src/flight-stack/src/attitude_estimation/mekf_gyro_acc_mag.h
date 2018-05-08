#ifndef MEKF_GYRO_ACC_MAG_H
#define MEKF_GYRO_ACC_MAG_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


class MEKFGyroAccMag
{
public:
    static const int STATE_DIM = 6; // attitude error a, gyro bias b
    static const int VECT_MEASURE_DIM = 2;

    Eigen::Quaternionf ref_attitude;

    Eigen::Matrix<float, STATE_DIM, 1> x;
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> P;
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> Q;
    Eigen::Matrix<float, VECT_MEASURE_DIM, VECT_MEASURE_DIM> R_mag;
    Eigen::Matrix<float, VECT_MEASURE_DIM, VECT_MEASURE_DIM> R_grav;

    MEKFGyroAccMag();
    ~MEKFGyroAccMag();

    void reset();
    void reset(Eigen::Quaternionf att);
    void update_imu(const float *gyro, const float *acc, float delta_t);
    void update_mag(const float *mag);
    Eigen::Quaternionf get_attitude() const;

    void apply_att_err_to_ref(void);

    /** Compute the transformation from inertial to measurement frame */
    static Eigen::Quaternionf vect_measurement_basis(Eigen::Vector3f expected);
    /** Compute the transformation from body to measurement from */
    Eigen::Matrix3f vect_measurement_basis_b_frame(Eigen::Vector3f expected) const;
    /** Compute the measurement z from a measureed vector in the body frame */
    static Eigen::Matrix<float, 2, 1> vect_measurement_transform(Eigen::Matrix3f measurement_basis, Eigen::Vector3f measurement);
    /** Compute the measurement sensitivity matrix with respect to attitude error a */
    static Eigen::Matrix<float, 2, 3> vect_measurement_jacobian(Eigen::Matrix3f measurement_basis, Eigen::Vector3f expected_b);
    void measure_vect(Eigen::Vector3f expected, Eigen::Vector3f measured, Eigen::Matrix2f R);
};

#endif /* MEKF_GYRO_ACC_MAG_H */
