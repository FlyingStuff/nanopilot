#ifndef EXTENDEDKALMANFILTER_H
#define EXTENDEDKALMANFILTER_H

#include <Eigen/Dense>

class ExtendedKalmanFilter
{
    typedef void (*state_prop_fn)(const Eigen::VectorXf &state,
                                  const Eigen::VectorXf &control,
                                  Eigen::VectorXf &out_state);

    typedef void (*state_prop_jacobian_fn)(const Eigen::VectorXf &state,
                                           const Eigen::VectorXf &control,
                                           Eigen::MatrixXf &out_jacobian);

    typedef void (*measurement_pred_fn)(const Eigen::VectorXf &state,
                                        Eigen::VectorXf &out_pred);

    typedef void (*measurement_pred_jacobian_fn)(const Eigen::VectorXf &state,
                                                 Eigen::MatrixXf &out_jacobian);

    state_prop_fn f;
    state_prop_jacobian_fn F;
    measurement_pred_fn h;
    measurement_pred_jacobian_fn H;
public:
    ExtendedKalmanFilter(unsigned int dimension,
                         state_prop_fn f,
                         state_prop_jacobian_fn F,
                         measurement_pred_fn h,
                         measurement_pred_jacobian_fn H);
    ~ExtendedKalmanFilter();

    void state_propagation(const Eigen::VectorXf &control_vect,
                           const Eigen::MatrixXf &control_cov);
    void measurement_update(const Eigen::VectorXf &measurement_vect,
                            const Eigen::MatrixXf &measurement_cov);
    void measurement_update(const Eigen::VectorXf &measurement_vect,
                            const Eigen::MatrixXf &measurement_cov,
                            measurement_pred_fn h,
                            measurement_pred_jacobian_fn H);
};

#endif /* EXTENDEDKALMANFILTER_H */