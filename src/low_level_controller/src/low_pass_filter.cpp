#include "low_pass_filter.hpp"

#define M_PI 3.141592f

float compute_alpha(float update_frequency, float cutoff_freq)
{
    float delta_t = 1/update_frequency;
    float fc = cutoff_freq;
    return (2*M_PI*delta_t*fc)/(2*M_PI*delta_t*fc+1);
}

LowPassFilter::LowPassFilter()
{
    m_prev_output = 0;
}

void LowPassFilter::set_update_frequency(float freq)
{
    m_update_freq = freq;
    m_alpha = compute_alpha(m_update_freq, parameter_scalar_get(&m_cutoff_freq));
}

void LowPassFilter::declare_parameters(parameter_namespace_t *parent_ns, const char *name)
{
    parameter_scalar_declare_with_default(&m_cutoff_freq, parent_ns, name, 1000.0f);
}

float LowPassFilter::process(float in)
{
    if (parameter_changed(&m_cutoff_freq)) {
        m_alpha = compute_alpha(m_update_freq, parameter_scalar_get(&m_cutoff_freq));
    }
    float y = m_alpha * in + (1-m_alpha) * m_prev_output;
    m_prev_output = y;
    return y;
}
