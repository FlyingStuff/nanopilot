#include "ekf_gyro_acc.h"
#include "CppUTest/TestHarness.h" // this include must be last


TEST_GROUP(EKFGyroAccTestGroup)
{
    EKFGyroAcc s;
    void setup(void)
    {
        s = EKFGyroAcc();
    }
};

TEST(EKFGyroAccTestGroup, UpdateIMU)
{
    float gyro[3] = {0, 0, 0};
    float acc[3] = {0, 0 ,0};
    s.update_imu(gyro, acc, 0.1);
}

TEST(EKFGyroAccTestGroup, GetAttitude)
{
    CHECK(s.get_attitude().isApprox(Eigen::Quaternionf(1, 0, 0, 0)));
}


TEST(EKFGyroAccTestGroup, NumericalMatlabExample)
{
    s.Q.setIdentity();
    s.R.setIdentity();
    s.x << 1, 0, 0, 0;
    s.P <<
   1.046143045332435,  -0.000372415678337,  -0.013552751498541,  -0.020068344308525,
  -0.000372415678337,   1.016114759720717,   0.012160901570127,   0.012897699979964,
  -0.013552751498541,   0.012160901570127,   1.042192271051624,  -0.011957165941359,
  -0.020068344308525,   0.012897699979964,  -0.011957165941359,   1.048208399567530;

    Eigen::Vector3f u;
    Eigen::Vector3f z;
    u << 1, 2, 3;
    z << 0.1000, 0.2000, 10.0000;
    s.update_imu(u.data(), z.data(), 0.001);

    Eigen::Matrix<float, 4, 4> P_expect;
    Eigen::Matrix<float, 4, 1> x_expect;
    x_expect <<
   0.999935314967245,
   0.010085747456303,
  -0.005034478301115,
   0.001515786211872;
    P_expect <<
   0.002599162161167,   0.000003067948055,  -0.000001545281543,  -0.003093427067918,
   0.000003067948055,   0.002596446714653,  -0.000000978863570,  -0.002028609310250,
  -0.000001545281543,  -0.000000978863570,   0.002594972887402,   0.001007119595048,
  -0.003093427067918,  -0.002028609310250,   0.001007119595048,   2.047853534697631;

    // std::cout << "x" << s.x << std::endl;
    // std::cout << "P" << s.P << std::endl;
    CHECK(s.x.isApprox(x_expect));
    CHECK(s.P.isApprox(P_expect));
}
