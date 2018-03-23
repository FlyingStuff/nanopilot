#include "rate_limiter.h"


void rate_limiter_init(rate_limiter_t *rl, const char *name, parameter_namespace_t *ns)
{
    parameter_scalar_declare_with_default(&rl->rate, ns, name, 0);
    rl->last_update = 0;
}

bool rate_limiter_should_run(rate_limiter_t *rl)
{
    bool should_run = true;
    timestamp_t now = timestamp_get();

    if (timestamp_duration(rl->last_update, now) >= 1.0f/parameter_scalar_read(&rl->rate)) {
        should_run = true;
        rl->last_update = now;
    } else {
        should_run = false;
    }
    return should_run;
}
