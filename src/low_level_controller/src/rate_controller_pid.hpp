#include "pid_with_parameter.hpp"
#include "low_pass_filter.hpp"
#include "control_loop.hpp"

class PIDRateController: public RateController {
    PIDController pid_roll_controller;
    PIDController pid_pitch_controller;
    PIDController pid_yaw_controller;
    LowPassFilter roll_lp;
    LowPassFilter pitch_lp;
    LowPassFilter yaw_lp;

public:
    PIDRateController();
    void declare_parameters(parameter_namespace_t *ns);
    virtual void process(const float rate_setpoint_rpy[3], const float rate_measured_rpy[3], float rate_ctrl_output_rpy[3]);
    virtual void set_update_frequency(float freq);
    virtual void reset();
};