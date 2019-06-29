#include "output_mixer_linear.hpp"

void LinearOutputMixer::mix(const float rate_ctrl_output_rpy[3], const struct rc_input_s &rc_inputs, const struct ap_ctrl_s &ap_ctrl, bool ap_control_en, std::array<float, NB_ACTUATORS> &output)
{
    std::array<float, NB_ACTUATORS> coeff;
    if (ap_control_en) {
        static_assert(NB_ACTUATORS <= MAX_NB_ACTUATORS);
        for (int i = 0; i < NB_ACTUATORS; i++) {
            output[i] = ap_ctrl.direct_output[i];
        }
        // force
        for (int axis=0; axis < 3; axis++) {
            parameter_vector_read(&m_force_ctrl_mix[axis], coeff.data());
            for (int i = 0; i < NB_ACTUATORS; i++) {
                output[i] += coeff[i] * ap_ctrl.force_xyz[axis];
            }
        }
    } else {
        // bias
        parameter_vector_read(&m_output_bias, coeff.data());
        for (int i = 0; i < NB_ACTUATORS; i++) {
            output[i] = coeff[i];
        }

        // rc_in
        assert(rc_inputs.nb_channels <= RC_INPUT_MAX_NB_CHANNELS);
        for (int in_idx = 0; in_idx < rc_inputs.nb_channels; in_idx++) {
            parameter_vector_read(&m_rc_mix[in_idx], coeff.data());
            for (int i = 0; i < NB_ACTUATORS; i++) {
                output[i] += coeff[i] * rc_inputs.channel[in_idx];
            }
        }
    }

    // roll, pitch, yaw torque
    for (int axis=0; axis < 3; axis++) {
        parameter_vector_read(&m_rpy_ctrl_mix[axis], coeff.data());
        for (int i = 0; i < NB_ACTUATORS; i++) {
            output[i] += coeff[i] * rate_ctrl_output_rpy[axis];
        }
    }
    if (ap_control_en) {
        for (int axis=0; axis < 3; axis++) {
            parameter_vector_read(&m_rpy_ctrl_mix[axis], coeff.data());
            for (int i = 0; i < NB_ACTUATORS; i++) {
                output[i] += coeff[i] * ap_ctrl.feed_forward_torque_rpy[axis];
            }
        }
    }
}

LinearOutputMixer::LinearOutputMixer()
{
    declare_parameters(NULL);
}

void LinearOutputMixer::declare_parameters(parameter_namespace_t *parent_ns)
{
    parameter_namespace_declare(&m_namespace, parent_ns, "LinearMixer");

    parameter_vector_declare_with_default(&m_rpy_ctrl_mix[0], &m_namespace, "roll", m_rpy_ctrl_mix_buf[0], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rpy_ctrl_mix[1], &m_namespace, "pitch", m_rpy_ctrl_mix_buf[1], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rpy_ctrl_mix[2], &m_namespace, "yaw", m_rpy_ctrl_mix_buf[2], NB_ACTUATORS);

    parameter_vector_declare_with_default(&m_force_ctrl_mix[0], &m_namespace, "fx", m_force_ctrl_mix_buf[0], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_force_ctrl_mix[1], &m_namespace, "fy", m_force_ctrl_mix_buf[1], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_force_ctrl_mix[2], &m_namespace, "fz", m_force_ctrl_mix_buf[2], NB_ACTUATORS);

    parameter_vector_declare_with_default(&m_output_bias, &m_namespace, "bias", m_output_bias_buf, NB_ACTUATORS);

    parameter_vector_declare_with_default(&m_rc_mix[0], &m_namespace, "rc_in1", m_rc_mix_buf[0], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[1], &m_namespace, "rc_in2", m_rc_mix_buf[1], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[2], &m_namespace, "rc_in3", m_rc_mix_buf[2], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[3], &m_namespace, "rc_in4", m_rc_mix_buf[3], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[4], &m_namespace, "rc_in5", m_rc_mix_buf[4], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[5], &m_namespace, "rc_in6", m_rc_mix_buf[5], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[6], &m_namespace, "rc_in7", m_rc_mix_buf[6], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[7], &m_namespace, "rc_in8", m_rc_mix_buf[7], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[8], &m_namespace, "rc_in9", m_rc_mix_buf[8], NB_ACTUATORS);
    parameter_vector_declare_with_default(&m_rc_mix[9], &m_namespace, "rc_in10", m_rc_mix_buf[9], NB_ACTUATORS);
}
