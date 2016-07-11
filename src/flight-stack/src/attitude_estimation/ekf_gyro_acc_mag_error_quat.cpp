#include "ekf_gyro_acc_mag_error_quat.h"
#include "template_kalman.h"

namespace code_gen {
    #include "code_gen/ekf_gyro_acc_mag_error_quat.h"
}

EKFGyroAccMagErrQuat::EKFGyroAccMagErrQuat()
{
    this->P.setIdentity();
    this->P *= 0.001;
    this->reset();
    this->Q = this->Q.setIdentity() * 0.000001;
    this->R = this->R.setIdentity() * 1000;
}

EKFGyroAccMagErrQuat::~EKFGyroAccMagErrQuat()
{

}

void EKFGyroAccMagErrQuat::update_imu(const float *gyro, const float *acc, float delta_t)
{
    Eigen::Map<const Eigen::Vector3f> u(gyro);
    Eigen::Map<const Eigen::Vector3f> acc_v(acc);
    // rotates inertial x to expected measurement (90deg pitch up)
    const float z_meas_ref[4] = {0.7071067812, 0, 0.7071067812, 0};
    Eigen::Matrix<float, 2, 1> z;
    code_gen::measurement_from_vect(acc_v.normalized().data(), this->q_ref.data(), z_meas_ref, z.data());

    // error quaternion transfer
    Eigen::Matrix<float, 4, 1> q_ref_cpy = this->q_ref;
    code_gen::apply_x_to_ref(q_ref_cpy.data(), this->x.data(), this->q_ref.data());
    // clear error quaternion
    this->x.block(0, 0, 3, 1).setZero();

    // reference propagation
    q_ref_cpy = this->q_ref;
    code_gen::ref_q_propagate(q_ref_cpy.data(), gyro, this->x.data(), delta_t, this->q_ref.data());
    // todo renormalize?

    auto f = [delta_t](Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, code_gen::CONTROL_DIM, 1> &control)
        {
            (void)state;
            (void)control;
            // Eigen::Matrix<float, code_gen::STATE_DIM, 1> state_cpy = state;
            // code_gen::f(state_cpy.data(), control.data(), delta_t, state.data());
        };
    auto F = [delta_t](const Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, code_gen::CONTROL_DIM, 1> &control,
                Eigen::Matrix<float, code_gen::STATE_DIM, code_gen::STATE_DIM> &out_jacobian)
        {
            code_gen::F(state.data(), control.data(), delta_t, out_jacobian.data());
        };
    auto h = [&, z_meas_ref](const Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
                Eigen::Matrix<float, code_gen::MEASURE_DIM, 1> &pred)
        {
            code_gen::h(state.data(), this->q_ref.data(), z_meas_ref, pred.data());
        };
    auto H = [&, z_meas_ref](const Eigen::Matrix<float, code_gen::STATE_DIM, 1> &state,
                Eigen::Matrix<float, code_gen::MEASURE_DIM, code_gen::STATE_DIM> &out_jacobian)
        {
            code_gen::H(state.data(), this->q_ref.data(), z_meas_ref, out_jacobian.data());
        };

    ekf_predict<float, code_gen::STATE_DIM, code_gen::CONTROL_DIM>(this->x, this->P, u, this->Q, f, F);
    ekf_measure<float, code_gen::STATE_DIM, code_gen::MEASURE_DIM>(this->x, this->P, z, this->R, h, H);
}

void EKFGyroAccMagErrQuat::reset()
{
    this->reset(Eigen::Quaternion<float>::Identity());
}

void EKFGyroAccMagErrQuat::reset(Eigen::Quaternionf att)
{
    this->x.setZero();
    this->q_ref(0, 0) = att.w();
    this->q_ref(1, 0) = att.x();
    this->q_ref(2, 0) = att.y();
    this->q_ref(3, 0) = att.z();
}

Eigen::Quaternionf EKFGyroAccMagErrQuat::get_attitude()
{
    return Eigen::Quaternionf(this->q_ref(0, 0),
                              this->q_ref(1, 0),
                              this->q_ref(2, 0),
                              this->q_ref(3, 0));
}
