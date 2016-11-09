#ifndef MESSAGEBUS_PORT_H
#define MESSAGEBUS_PORT_H


#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
// this is for the unit tests

typedef int msgbus_cond_t;
typedef int msgbus_mutex_t;

typedef struct {
    int lock;
    int cond;
} messagebus_condvar_wrapper_t;

void msgbus_unittest_assert(bool condition);
#define MSGBUS_ASSERT(x) msgbus_unittest_assert(x)


#else // ChibiOS

#include <ch.h>

typedef struct {
    mutex_t lock;
    condition_variable_t cond;
} messagebus_condvar_wrapper_t;

#endif


#ifdef __cplusplus
extern "C" {
#endif

void messagebus_condvar_wrapper_init(messagebus_condvar_wrapper_t *c);

#ifdef __cplusplus
}
#endif


#endif /* MESSAGEBUS_PORT_H */
