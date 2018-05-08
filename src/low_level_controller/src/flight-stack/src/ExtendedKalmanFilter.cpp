#include "ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(unsigned int dimension,
                                           state_prop_fn f,
                                           state_prop_jacobian_fn F,
                                           measurement_pred_fn h,
                                           measurement_pred_jacobian_fn H)
{
    this->f = f;
    this->F = F;
    this->h = h;
    this->H = H;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{

}

void ExtendedKalmanFilter::state_propagation(const Eigen::VectorXf &control_vect,
                                             const Eigen::MatrixXf &control_cov)
{
    Eigen::VectorXf new_state(control_vect.rows());
    this->f(control_vect, control_vect, new_state);
}