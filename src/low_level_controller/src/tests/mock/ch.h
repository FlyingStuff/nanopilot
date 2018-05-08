#ifndef CH_H
#define CH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

#define THD_WORKING_AREA(name, size) int name[1]
#define THD_FUNCTION(name, argname) void name(void *argname)

#define LOWPRIO 0

typedef int thread_t;
typedef int tprio_t;
typedef void (*tfunc_t)(void *p);

static inline
thread_t *chThdCreateStatic(void *wsp,
    size_t size,
    tprio_t prio,
    tfunc_t pf,
    void *arg)
{
    (void)wsp;
    (void)size;
    (void)prio;
    (void)pf;
    (void)arg;
    return NULL;
}

static inline
void chRegSetThreadName(const char *name)
{
    (void)name;
}


#ifdef __cplusplus
}
#endif

#endif /* CH_H */
