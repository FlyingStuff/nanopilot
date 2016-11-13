#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <assert.h>
#include "msgbus_port.h"


void msgbus_mutex_init(msgbus_mutex_t *mutex)
{
    assert(pthread_mutex_init(mutex, NULL) == 0);
}

void msgbus_mutex_acquire(msgbus_mutex_t *mutex)
{
    assert(pthread_mutex_lock(mutex) == 0);
}

void msgbus_mutex_release(msgbus_mutex_t *mutex)
{
    assert(pthread_mutex_unlock(mutex) == 0);
}

void msgbus_condvar_init(msgbus_cond_t *cond)
{
    assert(pthread_cond_init(cond, NULL) == 0);
}

void msgbus_condvar_broadcast(msgbus_cond_t *cond)
{
    assert(pthread_cond_broadcast(cond) == 0);
}

#ifdef __APPLE__ // OS X has no gettitme with realtime clock
void msgbus_condvar_wait(msgbus_cond_t *cond, msgbus_mutex_t *mutex, uint32_t timeout_us)
{
    struct timespec ts;
    ts.tv_sec = timeout_us / 1000000;
    ts.tv_nsec = (timeout_us % 1000000) * 1000;
    pthread_cond_timedwait_relative_np(cond, mutex, &ts);
}
#else // posix
void msgbus_condvar_wait(msgbus_cond_t *cond, msgbus_mutex_t *mutex, uint32_t timeout_us)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_us / 1000000;
    // ts.tv_nsec todo
    pthread_cond_timedwait(cond, mutex, &ts);
}
#endif

