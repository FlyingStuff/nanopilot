#include <Eigen/Dense>
#include "math_helpers.h"

Eigen::Matrix3f cross_product_matrix(Eigen::Vector3f v)
{
    Eigen::Matrix3f vx;
    vx <<     0, -v[2],  v[1],
           v[2],     0, -v[0],
          -v[1],  v[0],     0;
    return vx;
}
