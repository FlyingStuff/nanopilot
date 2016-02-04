#include "rate_limiter.h"


void rate_limiter_init(rate_limiter_t *rl, const char *name, parameter_namespace_t *ns)
{
    parameter_scalar_declare_with_default(&rl->rate, ns, name, 0);

}