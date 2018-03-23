#ifndef RATE_LIMITER_H
#define RATE_LIMITER_H

#include <parameter/parameter.h>
#include <timestamp.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * This module implements a rate limiter based on timestamps
 * The function rate_limiter_should_run returns whether the action should be run,
 * respecting the rate limit configured with the parameter service.
 * The parameter is a scalar which gives the update rate in Hz.
 * An update rate of zero or a negative value disables the update.
 * An update rate of INFINITY always runs.
 */


typedef struct {
    parameter_t rate;
    timestamp_t last_update;
} rate_limiter_t;


void rate_limiter_init(rate_limiter_t *rl, const char *name, parameter_namespace_t *ns);
bool rate_limiter_should_run(rate_limiter_t *rl);


#ifdef __cplusplus
}
#endif

#endif /* RATE_LIMITER_H */
