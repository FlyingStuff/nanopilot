#include <parameter/parameter.h>
#include "control_loop.hpp"

class LinearOutputMixer: public OutputMixer {
public:
    virtual void mix(const float rate_ctrl_output_rpy[3], const struct rc_input_s &rc_inputs, const struct ap_ctrl_s &ap_ctrl, bool ap_control_en, std::array<float, NB_ACTUATORS> &output);
    explicit LinearOutputMixer();
    void declare_parameters(parameter_namespace_t *parent_ns);
    virtual void set_update_frequency(float loop_frequency);

private:
    parameter_namespace_t m_namespace;
    float m_rpy_ctrl_mix_buf[3][NB_ACTUATORS] = {{0}};
    parameter_t m_rpy_ctrl_mix[3];
    float m_force_ctrl_mix_buf[3][NB_ACTUATORS] = {{0}};
    parameter_t m_force_ctrl_mix[3];
    float m_rc_mix_buf[RC_INPUT_MAX_NB_CHANNELS][NB_ACTUATORS] = {{0}};
    parameter_t m_rc_mix[RC_INPUT_MAX_NB_CHANNELS];
    float m_output_bias_buf[NB_ACTUATORS] = {0};
    parameter_t m_output_bias;
};