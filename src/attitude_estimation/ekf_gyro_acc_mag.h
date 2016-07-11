#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace _ekf_acc {
    #include "code_gen/ekf_gyro_acc.h"
}
namespace _ekf_mag {
    #include "code_gen/ekf_gyro_mag.h"
}

class EKFGyroAccMag
{
public:
    Eigen::Matrix<float, _ekf_acc::STATE_DIM, 1> x;
    Eigen::Matrix<float, _ekf_acc::STATE_DIM, _ekf_acc::STATE_DIM> P;
    Eigen::Matrix<float, _ekf_acc::STATE_DIM, _ekf_acc::STATE_DIM> Q;
    Eigen::Matrix<float, _ekf_acc::MEASURE_DIM, _ekf_acc::MEASURE_DIM> R_acc;
    Eigen::Matrix<float, _ekf_mag::MEASURE_DIM, _ekf_mag::MEASURE_DIM> R_mag;

    EKFGyroAccMag();
    ~EKFGyroAccMag();

    void reset();
    void reset(Eigen::Quaternionf att);
    void update_imu(const float *gyro, const float *acc, float delta_t);
    void update_mag(const float *mag);
    Eigen::Quaternionf get_attitude();

};
