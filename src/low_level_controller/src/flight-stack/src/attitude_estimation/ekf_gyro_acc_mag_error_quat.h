#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace _gyro_mag_error_q {
    #include "code_gen/ekf_gyro_acc_mag_error_quat.h"
}

class EKFGyroAccMagErrQuat
{
public:
    Eigen::Matrix<float, 4, 1> q_ref;

    Eigen::Matrix<float, _gyro_mag_error_q::STATE_DIM, 1> x;
    Eigen::Matrix<float, _gyro_mag_error_q::STATE_DIM, _gyro_mag_error_q::STATE_DIM> P;
    Eigen::Matrix<float, _gyro_mag_error_q::STATE_DIM, _gyro_mag_error_q::STATE_DIM> Q;
    Eigen::Matrix<float, _gyro_mag_error_q::MEASURE_DIM, _gyro_mag_error_q::MEASURE_DIM> R;

    EKFGyroAccMagErrQuat();
    ~EKFGyroAccMagErrQuat();

    void reset();
    void reset(Eigen::Quaternionf att);
    void update_imu(const float *gyro, const float *acc, float delta_t);
    Eigen::Quaternionf get_attitude();

};
