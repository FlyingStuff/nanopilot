#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

#include "msgbus/msgbus.hpp"
#include "rc_input.hpp"
#include "actuators.hpp"
#include <ros_interface/msg.h>
#include <Eigen/Dense>

extern msgbus::Topic<bool> output_armed_topic;
extern msgbus::Topic<bool> ap_in_control_topic;
extern msgbus::Topic<bool> ap_control_timeout;
extern msgbus::Topic<uint64_t> ap_control_latency_topic;
extern msgbus::Topic<struct ap_ctrl_s> ap_ctrl;
extern msgbus::Topic<std::array<float, NB_ACTUATORS>> actuator_output_topic;
extern msgbus::Topic<std::array<float, 3>> rate_setpoint_rpy_topic;
extern msgbus::Topic<std::array<float, 3>> rate_measured_rpy_topic;
extern msgbus::Topic<std::array<float, 3>> rate_ctrl_output_rpy_topic;
extern parameter_namespace_t control_ns;


class RateController {
public:
    virtual void process(const float rate_setpoint_rpy[3], const float rate_measured_rpy[3], float rate_ctrl_output_rpy[3]) = 0;
    virtual void set_update_frequency(float freq) = 0;
    virtual void reset() = 0;
};

class OutputMixer {
public:
    virtual void mix(const float rate_ctrl_output_rpy[3], const struct rc_input_s &rc_inputs, const struct ap_ctrl_s &ap_ctrl, bool ap_control_en, std::array<float, NB_ACTUATORS> &output) = 0;
};

void control_init();
void control_start(RateController &rate_ctrl, OutputMixer &output_mixer);

#endif /* CONTROL_LOOP_HPP */
