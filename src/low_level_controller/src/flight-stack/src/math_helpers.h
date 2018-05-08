#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#include <Eigen/Dense>

// some math functions missing in Eigen

Eigen::Matrix3f cross_product_matrix(Eigen::Vector3f v);

#endif /* MATH_HELPERS_H */