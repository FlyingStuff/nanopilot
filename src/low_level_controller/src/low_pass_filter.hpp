#include <parameter/parameter.h>

class LowPassFilter
{
public:
    LowPassFilter();
    void set_update_frequency(float freq);
    void declare_parameters(parameter_namespace_t *parent_ns, const char *name);
    float process(float in);

private:
    parameter_t m_cutoff_freq;
    float m_alpha=1;
    float m_update_freq;
    float m_prev_output;
};
