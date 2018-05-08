#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "CppUTestExt/MockSupport.h"
#include "CppUTest/TestHarness.h"
#include <iostream>

TEST_GROUP(Quaternion)
{

};

TEST(Quaternion, ConstructorOrder)
{
    Eigen::Quaternionf q(1, 2, 3, 4); // w, x, y, z
    CHECK_EQUAL(1, q.w());
    CHECK_EQUAL(2, q.x());
    CHECK_EQUAL(3, q.y());
    CHECK_EQUAL(4, q.z());
}

TEST(Quaternion, StorageOrder)
{
    Eigen::Quaternionf q(1, 2, 3, 4);
    float q_array[4];
    Eigen::Map<Eigen::Matrix<float, 4, 1> > (&q_array[0], 4) = q.coeffs();
    CHECK_EQUAL(2, q_array[0]); // x
    CHECK_EQUAL(3, q_array[1]); // y
    CHECK_EQUAL(4, q_array[2]); // z
    CHECK_EQUAL(1, q_array[3]); // w is last!
}

TEST(Quaternion, Multiplication)
{
    Eigen::Quaternionf a(1, 2, 3, 4);
    Eigen::Quaternionf b(5, 6, 7, 8);
    Eigen::Quaternionf ab = a*b;
    float ab_exp0 = a.w()*b.w() - a.vec().dot(b.vec());
    Eigen::Vector3f ab_expv = a.w()*b.vec() + b.w()*a.vec() + a.vec().cross(b.vec());
    CHECK_EQUAL(ab_exp0, ab.w());
    CHECK_EQUAL(ab_expv[0], ab.x());
    CHECK_EQUAL(ab_expv[1], ab.y());
    CHECK_EQUAL(ab_expv[2], ab.z());
}

TEST(Quaternion, RotationMatrix)
{
    float t = 0.1;
    Eigen::Quaternionf q(cos(t/2), 0, 0, sin(t/2));
    Eigen::Matrix3f rq = q.toRotationMatrix();
    Eigen::Matrix3f rq_exp;
    rq_exp << cos(t), -sin(t), 0,
              sin(t), cos(t), 0,
              0, 0, 1;
    CHECK_TRUE(rq_exp.isApprox(rq));
}

TEST(Quaternion, fromAxisAngle)
{
    float t = 0.1;
    Eigen::Quaternionf q(Eigen::AngleAxisf(t, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf q_exp(cos(t/2), 0, 0, sin(t/2));
    CHECK_TRUE(q_exp.isApprox(q));
}
