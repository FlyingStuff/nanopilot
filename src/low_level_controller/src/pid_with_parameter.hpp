#include <parameter/parameter.h>
#include "pid.h"

class PIDController
{
public:
    PIDController();
    void set_update_frequency(float freq);
    void declare_parameters(parameter_namespace_t *parent_ns, const char *name);
    float process(float error);
    void reset();

private:
    parameter_namespace_t m_namespace;
    parameter_t m_kp;
    parameter_t m_ki;
    parameter_t m_kd;
    parameter_t m_integrator_output_limit;
    pid_ctrl_t m_pid;
};
