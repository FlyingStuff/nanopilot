#include "ekf_gyro_acc.h"
#include "template_kalman.h"

namespace gyro_acc {
    #include "code_gen/ekf_gyro_acc.h"
}

EKFGyroAcc::EKFGyroAcc()
{
    this->P.setIdentity();
    this->P *= 0.001;
    this->reset();
    this->Q = this->Q.setIdentity() * 0.000001;
    this->R = this->R.setIdentity() * 1000;
}

EKFGyroAcc::~EKFGyroAcc()
{

}

void EKFGyroAcc::update_imu(const float *gyro, const float *acc, float delta_t)
{
    Eigen::Map<const Eigen::Vector3f> u(gyro);
    Eigen::Map<const Eigen::Vector3f> z(acc);

    auto f = [delta_t](Eigen::Matrix<float, gyro_acc::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, gyro_acc::CONTROL_DIM, 1> &control)
        {
            Eigen::Matrix<float, gyro_acc::STATE_DIM, 1> state_cpy = state;
            gyro_acc::f(state_cpy.data(), control.data(), delta_t, state.data());
        };
    auto F = [delta_t](const Eigen::Matrix<float, gyro_acc::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, gyro_acc::CONTROL_DIM, 1> &control,
                Eigen::Matrix<float, gyro_acc::STATE_DIM, gyro_acc::STATE_DIM> &out_jacobian)
        {
            gyro_acc::F(state.data(), control.data(), delta_t, out_jacobian.data());
        };
    auto h = [](const Eigen::Matrix<float, gyro_acc::STATE_DIM, 1> &state,
                Eigen::Matrix<float, gyro_acc::MEASURE_DIM, 1> &pred)
        {
            gyro_acc::h(state.data(), pred.data());
        };
    auto H = [](const Eigen::Matrix<float, gyro_acc::STATE_DIM, 1> &state,
                Eigen::Matrix<float, gyro_acc::MEASURE_DIM, gyro_acc::STATE_DIM> &out_jacobian)
        {
            gyro_acc::H(state.data(), out_jacobian.data());
        };

    ekf_predict<float, gyro_acc::STATE_DIM, gyro_acc::CONTROL_DIM>(this->x, this->P, u, this->Q, f, F);
    this->x.topLeftCorner(4, 1) = this->x.topLeftCorner(4, 1).normalized();
    ekf_measure<float, gyro_acc::STATE_DIM, gyro_acc::MEASURE_DIM>(this->x, this->P, z, this->R, h, H);
    this->x.topLeftCorner(4, 1) = this->x.topLeftCorner(4, 1).normalized();
}

void EKFGyroAcc::reset()
{
    this->reset(Eigen::Quaternion<float>::Identity());
}

void EKFGyroAcc::reset(Eigen::Quaternionf att)
{
    this->x.setZero();
    this->x(0, 0) = att.w();
    this->x(1, 0) = att.x();
    this->x(2, 0) = att.y();
    this->x(3, 0) = att.z();
}

Eigen::Quaternionf EKFGyroAcc::get_attitude()
{
    return Eigen::Quaternionf(this->x(0, 0), this->x(1, 0), this->x(2, 0), this->x(3, 0));
}
