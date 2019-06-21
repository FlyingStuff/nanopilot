#include "pid_with_parameter.hpp"

PIDController::PIDController()
{
    pid_init(&m_pid);
    parameter_namespace_declare(&m_namespace, NULL, NULL);
    parameter_scalar_declare(&m_kp, &m_namespace, "Kp");
    parameter_scalar_declare(&m_ki, &m_namespace, "Ki");
    parameter_scalar_declare(&m_kd, &m_namespace, "Kd");
    parameter_scalar_declare(&m_integrator_output_limit, &m_namespace, "I_out_limit");
}

void PIDController::set_update_frequency(float freq)
{
    pid_set_frequency(&m_pid, freq);
}

void PIDController::declare_parameters(parameter_namespace_t *parent_ns, const char *name)
{
    parameter_namespace_declare(&m_namespace, parent_ns, name);
    parameter_scalar_declare_with_default(&m_kp, &m_namespace, "Kp", 1);
    parameter_scalar_declare_with_default(&m_ki, &m_namespace, "Ki", 0);
    parameter_scalar_declare_with_default(&m_kd, &m_namespace, "Kd", 0);
    parameter_scalar_declare_with_default(&m_integrator_output_limit, &m_namespace, "I_out_limit", 1);
}

float PIDController::process(float error)
{
    if (parameter_namespace_contains_changed(&m_namespace)) {
        if (parameter_changed(&m_kp) || parameter_changed(&m_ki) || parameter_changed(&m_kd)) {
            pid_set_gains(&m_pid, parameter_scalar_get(&m_kp), parameter_scalar_get(&m_ki), parameter_scalar_get(&m_kd));
        }
        if (parameter_changed(&m_integrator_output_limit)) {
            pid_set_integrator_output_limit(&m_pid, parameter_scalar_get(&m_integrator_output_limit));
        }
    }

    return pid_process(&m_pid, error);
}

void PIDController::reset()
{
    pid_reset_integral(&m_pid);
}
