#ifndef TEMPLATE_KALMAN_H
#define TEMPLATE_KALMAN_H

#include <functional>
#include <Eigen/Dense>

template<typename scalar, int state_dim>
void ekf_predict(Eigen::Matrix<scalar, state_dim, state_dim> &state_cov,
                 const Eigen::Matrix<scalar, state_dim, state_dim> &process_noise_cov,
                 const Eigen::Matrix<scalar, state_dim, state_dim> &phi
                 )
{
    state_cov = phi * state_cov * phi.transpose() + process_noise_cov;
}


template<typename scalar, int state_dim, int control_dim>
void ekf_predict(Eigen::Matrix<scalar, state_dim, 1> &state,
                 Eigen::Matrix<scalar, state_dim, state_dim> &state_cov,
                 const Eigen::Matrix<scalar, control_dim, 1> &control,
                 const Eigen::Matrix<scalar, state_dim, state_dim> &process_noise_cov,
                 std::function<void (Eigen::Matrix<scalar, state_dim, 1> &state,
                                     const Eigen::Matrix<scalar, control_dim, 1> &control)> state_prop_fn,
                 std::function<void (const Eigen::Matrix<scalar, state_dim, 1> &state,
                                     const Eigen::Matrix<scalar, control_dim, 1> &control,
                                     Eigen::Matrix<scalar, state_dim, state_dim> &out_jacobian)> state_prop_jacobian_fn
                 )
{
    Eigen::Matrix<scalar, state_dim, state_dim> phi;
    state_prop_jacobian_fn(state, control, phi);
    state_prop_fn(state, control);
    ekf_predict<scalar, state_dim>(state_cov, process_noise_cov, phi);
}


template<typename scalar, int state_dim, int measurement_dim>
void ekf_measure(Eigen::Matrix<scalar, state_dim, 1> &state,
                 Eigen::Matrix<scalar, state_dim, state_dim> &state_cov,
                 const Eigen::Matrix<scalar, measurement_dim, 1> &measurement,
                 const Eigen::Matrix<scalar, measurement_dim, measurement_dim> &measurement_cov,
                 const Eigen::Matrix<scalar, measurement_dim, 1> &measurement_prediction,
                 const Eigen::Matrix<scalar, measurement_dim, state_dim> &H
                 )
{
    Eigen::Matrix<scalar, measurement_dim, 1> y;
    Eigen::Matrix<scalar, measurement_dim, measurement_dim> S;
    Eigen::Matrix<scalar, state_dim, measurement_dim> K;

    y = measurement - measurement_prediction;
    S = H * state_cov * H.transpose() + measurement_cov;
    K = state_cov * H.transpose() * S.inverse();

    Eigen::Matrix<scalar, state_dim, state_dim> I;
    I.setIdentity();
    state_cov = (I - K * H) * state_cov;
    state = state + K * y;
}


template<typename scalar, int state_dim, int measurement_dim>
void ekf_measure(Eigen::Matrix<scalar, state_dim, 1> &state,
                 Eigen::Matrix<scalar, state_dim, state_dim> &state_cov,
                 const Eigen::Matrix<scalar, measurement_dim, 1> &measurement,
                 const Eigen::Matrix<scalar, measurement_dim, measurement_dim> &measurement_cov,
                 std::function<void (const Eigen::Matrix<scalar, state_dim, 1> &state,
                                     Eigen::Matrix<scalar, measurement_dim, 1> &out_pred)> measurement_pred_fn,
                 std::function<void (const Eigen::Matrix<scalar, state_dim, 1> &state,
                                     Eigen::Matrix<scalar, measurement_dim, state_dim> &out_jacobian)> measurement_pred_jacobian_fn
                 )
{
    Eigen::Matrix<scalar, measurement_dim, 1> h_x;
    Eigen::Matrix<scalar, measurement_dim, state_dim> H;
    measurement_pred_fn(state, h_x);
    measurement_pred_jacobian_fn(state, H);

    ekf_measure<scalar, state_dim, measurement_dim>(
            state, state_cov, measurement, measurement_cov, h_x, H);
}

#endif /* TEMPLATE_KALMAN_H */