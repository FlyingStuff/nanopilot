#include "mekf_gyro_acc_mag.h"
#include "CppUTest/TestHarness.h" // this include must be last
#include <iostream>

TEST_GROUP(MEKFGyroAccMagTestGroup)
{
    MEKFGyroAccMag k;
    void setup(void)
    {
        k = MEKFGyroAccMag();
    }
};


TEST(MEKFGyroAccMagTestGroup, apply_att_err_to_ref)
{
    k.x[0] = 0.01;
    k.x[1] = 0.02;
    k.x[2] = 0.03;
    k.x.block(3, 0, 3, 1) = Eigen::Vector3f(1,2,3); // gyro bias
    Eigen::Quaternionf ref_att_before_update(cos(0.1), 0, 0, sin(0.1));
    k.ref_attitude = ref_att_before_update;

    k.apply_att_err_to_ref();

    // attitude error is correctly reset
    CHECK_TRUE(k.x.block(0, 0, 3, 1).isZero());
    // error is transferred to the reference attitude
    Eigen::Quaternionf q_of_a = Eigen::Quaternionf(2, 0.01, 0.02, 0.03).normalized();
    Eigen::Quaternionf expected_att = ref_att_before_update * q_of_a;
    CHECK_TRUE(k.ref_attitude.isApprox(expected_att));
    // bias is left unchanged
    CHECK_TRUE(k.x.block(3, 0, 3, 1).isApprox(Eigen::Vector3f(1,2,3)));
}

TEST(MEKFGyroAccMagTestGroup, vect_measurement_basis)
{
    Eigen::Vector3f v_i(1, 2, 3); // expectation of v in inertial frame
    Eigen::Quaternionf q = MEKFGyroAccMag::vect_measurement_basis(v_i);
    Eigen::Vector3f v_m = q._transformVector(v_i.normalized());
    // the expectation of the measurement is [1,0,0]' in the meas. frame
    CHECK_TRUE(v_m.isApprox(Eigen::Vector3f(1, 0, 0)));
}

TEST(MEKFGyroAccMagTestGroup, vect_measurement_basis_b_frame)
{
    Eigen::Quaternionf att(Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f v_i(1, 2, 3); // expectation of v in inertial frame
    k.reset(att);
    Eigen::Matrix3f b_to_m = k.vect_measurement_basis_b_frame(v_i);
    Eigen::Vector3f v_b = att.conjugate()._transformVector(v_i);
    Eigen::Vector3f v_m = b_to_m * v_b;
    // the expectation of the measurement is [1,0,0]' in the meas. frame
    CHECK_TRUE(v_m.normalized().isApprox(Eigen::Vector3f(1, 0, 0)));
}

TEST(MEKFGyroAccMagTestGroup, vect_measurement_transform)
{
    Eigen::Quaternionf att(Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f v_i(1, 2, 3); // expectation of v in inertial frame
    k.reset(att);
    Eigen::Matrix3f b_to_m = k.vect_measurement_basis_b_frame(v_i);
    Eigen::Vector3f v_b = att.conjugate()._transformVector(v_i);

    Eigen::Vector2f z = MEKFGyroAccMag::vect_measurement_transform(b_to_m, v_b);

    // the measurement z of the expectation is zero
    CHECK_TRUE(z.isZero());
}

TEST(MEKFGyroAccMagTestGroup, vect_measurement_jacobian)
{
    Eigen::Quaternionf att(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()));
    Eigen::Vector3f v_i(0, 1, 0); // expectation of v in inertial frame
    // expectation of v is along [0,1,0] in inertial and [1,0,0] in body frame,
    // because the body frame is oriented +90deg yaw
    k.reset(att);
    Eigen::Vector3f v_b = att.conjugate()._transformVector(v_i); // [1,0,0]
    Eigen::Matrix3f b_to_m = k.vect_measurement_basis_b_frame(v_i);
    //std::cout << std::endl << v_b << std::endl;
    Eigen::Matrix<float, 2, 3> Ha = MEKFGyroAccMag::vect_measurement_jacobian(b_to_m, v_b);
    Eigen::Matrix<float, 2, 3> Ha_exp;
    // sensitivity of z (= last 2 components of v_m) with respect to a:
    //  with respect to a1 (=roll right) = [0, 0]' (v_b is [1, 0, 0], along roll axis)
    //  with respect to a2 (=pitch up) = [0, 1]' (v_b goes down (positive))
    //  with respect to a3 (=yaw right) = [-1, 0]' (v_b goes left (negative))
    Ha_exp << 0, 0, -1,
              0, 1,  0;
    //std::cout << std::endl << Ha << std::endl;
    CHECK_TRUE(Ha.isApprox(Ha_exp));
}
