#define EIGEN_RUNTIME_NO_MALLOC
#include <Eigen/Dense>
#include "math_helpers.h"
#include "CppUTestExt/MockSupport.h"
#include "CppUTest/TestHarness.h"


TEST_GROUP(Math)
{

};

TEST(Math, cross_product_matrix)
{
    Eigen::Matrix3f xm = cross_product_matrix(Eigen::Vector3f(1, 2, 3));
    Eigen::Matrix3f xm_exp;
    xm_exp << 0, -3,  2,
              3,  0, -1,
             -2,  1,  0;
    CHECK_TRUE(xm.isApprox(xm_exp));
}

TEST(Math, cross_product_matrix_same_as_cross_product)
{
    Eigen::Vector3f a(1, 2, 3);
    Eigen::Vector3f b(4, 5, 6);
    Eigen::Matrix3f ax = cross_product_matrix(a);
    CHECK_TRUE((ax*b).isApprox(a.cross(b)));
}
