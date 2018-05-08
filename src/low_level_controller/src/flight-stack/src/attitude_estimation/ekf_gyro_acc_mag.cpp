#include "ekf_gyro_acc_mag.h"
#include "template_kalman.h"

namespace ekf_acc {
    #include "code_gen/ekf_gyro_acc.h"
}
namespace ekf_mag {
    #include "code_gen/ekf_gyro_mag.h"
}

EKFGyroAccMag::EKFGyroAccMag()
{
    this->P.setIdentity();
    this->P *= 0.001;
    this->reset();
    this->Q = this->Q.setIdentity() * 0.00001;
    this->R_acc = this->R_acc.setIdentity() * 1000;
    this->R_mag = this->R_mag.setIdentity() * 10;
}

EKFGyroAccMag::~EKFGyroAccMag()
{

}

void EKFGyroAccMag::update_imu(const float *gyro, const float *acc, float delta_t)
{
    Eigen::Map<const Eigen::Vector3f> u(gyro);
    Eigen::Map<const Eigen::Vector3f> z(acc);

    auto f = [delta_t](Eigen::Matrix<float, ekf_acc::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, ekf_acc::CONTROL_DIM, 1> &control)
        {
            Eigen::Matrix<float, ekf_acc::STATE_DIM, 1> state_cpy = state;
            ekf_acc::f(state_cpy.data(), control.data(), delta_t, state.data());
        };
    auto F = [delta_t](const Eigen::Matrix<float, ekf_acc::STATE_DIM, 1> &state,
                const Eigen::Matrix<float, ekf_acc::CONTROL_DIM, 1> &control,
                Eigen::Matrix<float, ekf_acc::STATE_DIM, ekf_acc::STATE_DIM> &out_jacobian)
        {
            ekf_acc::F(state.data(), control.data(), delta_t, out_jacobian.data());
        };
    auto h = [](const Eigen::Matrix<float, ekf_acc::STATE_DIM, 1> &state,
                Eigen::Matrix<float, ekf_acc::MEASURE_DIM, 1> &pred)
        {
            ekf_acc::h(state.data(), pred.data());
        };
    auto H = [](const Eigen::Matrix<float, ekf_acc::STATE_DIM, 1> &state,
                Eigen::Matrix<float, ekf_acc::MEASURE_DIM, ekf_acc::STATE_DIM> &out_jacobian)
        {
            ekf_acc::H(state.data(), out_jacobian.data());
        };

    ekf_predict<float, ekf_acc::STATE_DIM, ekf_acc::CONTROL_DIM>(this->x, this->P, u, this->Q, f, F);
    this->x.topLeftCorner(4, 1) = this->x.topLeftCorner(4, 1).normalized();
    ekf_measure<float, ekf_acc::STATE_DIM, ekf_acc::MEASURE_DIM>(this->x, this->P, z, this->R_acc, h, H);
    this->x.topLeftCorner(4, 1) = this->x.topLeftCorner(4, 1).normalized();
}

void EKFGyroAccMag::update_mag(const float *mag)
{
    float mag_heading;
    ekf_mag::meas_transf(this->x.topLeftCorner(4, 1).data(), mag, &mag_heading);

    Eigen::Map<const Eigen::Matrix<float, 1, 1>> z(&mag_heading);

    auto h = [](const Eigen::Matrix<float, ekf_mag::STATE_DIM, 1> &state,
                Eigen::Matrix<float, ekf_mag::MEASURE_DIM, 1> &pred)
        {
            ekf_mag::h(state.data(), pred.data());
        };
    auto H = [](const Eigen::Matrix<float, ekf_mag::STATE_DIM, 1> &state,
                Eigen::Matrix<float, ekf_mag::MEASURE_DIM, ekf_mag::STATE_DIM> &out_jacobian)
        {
            ekf_mag::H(state.data(), out_jacobian.data());
        };

    ekf_measure<float, ekf_mag::STATE_DIM, ekf_mag::MEASURE_DIM>(this->x, this->P, z, this->R_mag, h, H);
    this->x.topLeftCorner(4, 1) = this->x.topLeftCorner(4, 1).normalized();
}

void EKFGyroAccMag::reset()
{
    this->reset(Eigen::Quaternion<float>::Identity());
}

void EKFGyroAccMag::reset(Eigen::Quaternionf att)
{
    this->x.setZero();
    this->x(0, 0) = att.w();
    this->x(1, 0) = att.x();
    this->x(2, 0) = att.y();
    this->x(3, 0) = att.z();
}

Eigen::Quaternionf EKFGyroAccMag::get_attitude()
{
    return Eigen::Quaternionf(this->x(0, 0), this->x(1, 0), this->x(2, 0), this->x(3, 0));
}
