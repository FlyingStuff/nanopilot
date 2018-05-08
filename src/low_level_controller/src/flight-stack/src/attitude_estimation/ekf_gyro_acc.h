#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace _gyro_acc {
    #include "code_gen/ekf_gyro_acc.h"
}

class EKFGyroAcc
{
public:
    Eigen::Matrix<float, _gyro_acc::STATE_DIM, 1> x;
    Eigen::Matrix<float, _gyro_acc::STATE_DIM, _gyro_acc::STATE_DIM> P;
    Eigen::Matrix<float, _gyro_acc::STATE_DIM, _gyro_acc::STATE_DIM> Q;
    Eigen::Matrix<float, _gyro_acc::MEASURE_DIM, _gyro_acc::MEASURE_DIM> R;

    EKFGyroAcc();
    ~EKFGyroAcc();

    void reset();
    void reset(Eigen::Quaternionf att);
    void update_imu(const float *gyro, const float *acc, float delta_t);
    Eigen::Quaternionf get_attitude();

};
