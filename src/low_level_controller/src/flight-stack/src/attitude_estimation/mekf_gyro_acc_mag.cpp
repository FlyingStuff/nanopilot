#include "template_kalman.h"
#include "math_helpers.h"
#include "mekf_gyro_acc_mag.h"


MEKFGyroAccMag::MEKFGyroAccMag()
{
    this->reset();
    this->Q = this->Q.setIdentity() * 0.000001;
    this->R_mag = this->R_mag.setIdentity() * 1000;
    this->R_grav = this->R_grav.setIdentity() * 1000;
}

MEKFGyroAccMag::~MEKFGyroAccMag()
{

}

void MEKFGyroAccMag::update_imu(const float *gyro, const float *acc, float delta_t)
{
    Eigen::Map<const Eigen::Vector3f> gyro_vec(gyro);
    Eigen::Map<const Eigen::Vector3f> acc_vec(acc);

    Eigen::Vector3f omega_ref = gyro_vec - this->x.block(3, 0, 3, 1); // gyro - bias

    // reference propagation
    float angle = omega_ref.norm() * delta_t;
    Eigen::Vector3f axis = omega_ref.normalized();
    this->ref_attitude = this->ref_attitude * Eigen::Quaternionf(Eigen::AngleAxisf(angle, axis));

    // exp x_dot = 0, so x is constant

    Eigen::Matrix<float, STATE_DIM, STATE_DIM> F;
    F.setZero();
    F.block(0, 0, 3, 3) = -cross_product_matrix(omega_ref); // jacobian(a_dot, a)
    F.block(0, 3, 3, 3) = -Eigen::Matrix3f::Identity(); // jacobian(a_dot, b)
    Eigen::Matrix<float, STATE_DIM, STATE_DIM> Phi = Eigen::Matrix<float, STATE_DIM, STATE_DIM>::Identity() + F * delta_t;

    ekf_predict<float, STATE_DIM>(this->P, this->Q, Phi);

    this->measure_vect(Eigen::Vector3f(0, 0, -1), acc_vec, this->R_grav);

    // error quaternion transfer
    this->apply_att_err_to_ref();
}

void MEKFGyroAccMag::update_mag(const float *mag)
{
    (void)mag;
    // todo we need a magnetic field model
}

void MEKFGyroAccMag::reset()
{
    this->reset(Eigen::Quaternion<float>::Identity());
}

void MEKFGyroAccMag::reset(Eigen::Quaternionf att)
{
    this->ref_attitude = att;
    this->x.setZero();
    this->P.setIdentity();
    this->P *= 0.001;
}

Eigen::Quaternionf MEKFGyroAccMag::get_attitude() const
{
    return this->ref_attitude;
}

void MEKFGyroAccMag::apply_att_err_to_ref(void)
{
    Eigen::Quaternionf delta_q_of_a(2, this->x[0], this->x[1], this->x[2]); // this is unnormalized!
    this->ref_attitude = this->ref_attitude * delta_q_of_a;
    this->ref_attitude.normalize(); // normalize after multiplication
    this->x.block(0, 0, 3, 1).setZero();
}

Eigen::Quaternionf MEKFGyroAccMag::vect_measurement_basis(Eigen::Vector3f expected)
{
    Eigen::Quaternionf transformation;
    transformation.setFromTwoVectors(expected, Eigen::Vector3f(1, 0, 0));
    return transformation;
}

Eigen::Matrix3f MEKFGyroAccMag::vect_measurement_basis_b_frame(Eigen::Vector3f expected) const
{
    Eigen::Quaternionf I_to_m = MEKFGyroAccMag::vect_measurement_basis(expected);
    return (I_to_m * this->ref_attitude).toRotationMatrix();
}

Eigen::Matrix<float, 2, 1> MEKFGyroAccMag::vect_measurement_transform(Eigen::Matrix3f measurement_basis, Eigen::Vector3f measurement_b)
{
    Eigen::Matrix<float, 2, 3>v_m_to_z;
    v_m_to_z << 0, 1, 0,
                0, 0, 1;
    return v_m_to_z * measurement_basis * measurement_b.normalized();
}

Eigen::Matrix<float, 2, 3> MEKFGyroAccMag::vect_measurement_jacobian(Eigen::Matrix3f measurement_basis, Eigen::Vector3f expected_b)
{
    Eigen::Matrix<float, 2, 3>v_m_to_z;
    v_m_to_z << 0, 1, 0,
                0, 0, 1;
    return v_m_to_z * measurement_basis * cross_product_matrix(expected_b.normalized());
}

void MEKFGyroAccMag::measure_vect(Eigen::Vector3f expected, Eigen::Vector3f measured, Eigen::Matrix2f R)
{
    Eigen::Matrix3f b_to_m = this->vect_measurement_basis_b_frame(expected);
    Eigen::Vector3f expected_b = this->ref_attitude.conjugate()._transformVector(expected);
    Eigen::Vector2f z = vect_measurement_transform(b_to_m, measured);
    Eigen::Matrix<float, VECT_MEASURE_DIM, 3> Ha = vect_measurement_jacobian(b_to_m, expected_b);
    Eigen::Matrix<float, VECT_MEASURE_DIM, STATE_DIM> H;
    H.setZero();
    H.block(0, 0, VECT_MEASURE_DIM, 3) = Ha; // measurement only depends on a

    ekf_measure<float, STATE_DIM, VECT_MEASURE_DIM>(this->x,
                                                    this->P,
                                                    z,
                                                    R,
                                                    Eigen::Vector2f::Zero(),
                                                    H);
}

