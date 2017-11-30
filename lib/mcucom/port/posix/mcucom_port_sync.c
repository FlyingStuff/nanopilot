#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include "mcucom_port_sync.h"


void mcucom_port_mutex_init(mcucom_port_mutex_t *mutex)
{
    assert(pthread_mutex_init(mutex, NULL) == 0);
}

void mcucom_port_mutex_acquire(mcucom_port_mutex_t *mutex)
{
    assert(pthread_mutex_lock(mutex) == 0);
}

void mcucom_port_mutex_release(mcucom_port_mutex_t *mutex)
{
    assert(pthread_mutex_unlock(mutex) == 0);
}

void mcucom_port_condvar_init(mcucom_port_cond_t *cond)
{
    assert(pthread_cond_init(cond, NULL) == 0);
}

void mcucom_port_condvar_broadcast(mcucom_port_cond_t *cond)
{
    assert(pthread_cond_broadcast(cond) == 0);
}

#ifdef __APPLE__ // OS X has no gettime with realtime clock
bool mcucom_port_condvar_wait(mcucom_port_cond_t *cond, mcucom_port_mutex_t *mutex, uint32_t timeout_us)
{
    if (timeout_us == MCUCOM_PORT_TIMEOUT_IMMEDIATE) {
        return false;
    }
    if (timeout_us == MCUCOM_PORT_TIMEOUT_NEVER) {
        assert(pthread_cond_wait(cond, mutex) == 0);
    } else {
        struct timespec ts;
        ts.tv_sec = timeout_us / 1000000;
        ts.tv_nsec = (timeout_us % 1000000) * 1000;
        int ret = pthread_cond_timedwait_relative_np(cond, mutex, &ts);
        assert(ret == 0 || ret == ETIMEDOUT);
        if (ret == ETIMEDOUT) {
            return false;
        }
    }
    return true;
}
#else // posix
bool mcucom_port_condvar_wait(mcucom_port_cond_t *cond, mcucom_port_mutex_t *mutex, uint32_t timeout_us)
{
    if (timeout_us == MCUCOM_PORT_TIMEOUT_IMMEDIATE) {
        return false;
    }
    if (timeout_us == MCUCOM_PORT_TIMEOUT_NEVER) {
        assert(pthread_cond_wait(cond, mutex) == 0);
    } else {
        struct timespec ts;
        assert(clock_gettime(CLOCK_REALTIME, &ts) == 0);
        ts.tv_nsec += (timeout_us % 1000000) * 1000;
        ts.tv_sec += timeout_us / 1000000 + ts.tv_nsec / 1000000000;
        ts.tv_nsec = ts.tv_nsec % 1000000000;
        int ret = pthread_cond_timedwait(cond, mutex, &ts);
        assert(ret == 0 || ret == ETIMEDOUT);
        if (ret == ETIMEDOUT) {
            return false;
        }
    }
    return true;
}
#endif
