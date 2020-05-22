#ifndef MSGBUS_PORT_HPP
#define MSGBUS_PORT_HPP


#if defined (MSGBUS_UNITTEST)
// Plaform implmentation for unittests

namespace msgbus {

void _lock();
void _unlock();
typedef int condvar_t;
void condvar_init(condvar_t *cv);
void condvar_destroy(condvar_t *cv);
void condvar_broadcast(condvar_t *cv);
bool condvar_wait(condvar_t *cv, float timeout);

}

#elif defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
// Posix Platform

#include <pthread.h>
#include <time.h>
#include <errno.h>

namespace msgbus {

inline pthread_mutex_t *get_mutex()
{
    static pthread_mutex_t mutex;
    static bool is_initialized = false;
    if (!is_initialized) {
        pthread_mutex_init(&mutex, NULL);
        is_initialized = true;
    }
    return &mutex;
}

inline void _lock()
{
    assert(pthread_mutex_lock(get_mutex()) == 0);
}

inline void _unlock()
{
    assert(pthread_mutex_unlock(get_mutex()) == 0);
}

typedef pthread_cond_t condvar_t;

inline void condvar_init(condvar_t *cv)
{
    assert(pthread_cond_init(cv, NULL) == 0);
}

inline void condvar_destroy(condvar_t *cv)
{
    assert(pthread_cond_destroy(cv) == 0);
}

inline void condvar_broadcast(condvar_t *cv)
{
    assert(pthread_cond_broadcast(cv) == 0);
}


#ifdef __APPLE__ // OS X has no gettime with realtime clock
inline bool condvar_wait(condvar_t *cv, float timeout)
{
    if (timeout == 0) {
        return false;
    }
    if (timeout < 0) {
        assert(pthread_cond_wait(cv, get_mutex()) == 0);
    } else {
        struct timespec ts;
        ts.tv_sec = timeout;
        ts.tv_nsec = (timeout - ts.tv_sec) * 1000000000UL;
        int ret = pthread_cond_timedwait_relative_np(cv, get_mutex(), &ts);
        assert(ret == 0 || ret == ETIMEDOUT);
        if (ret == ETIMEDOUT) {
            return false;
        }
    }
    return true;
}
#else // posix
inline bool condvar_wait(condvar_t *cv, float timeout)
{
    if (timeout == 0) {
        return false;
    }
    if (timeout < 0) {
        assert(pthread_cond_wait(cv, get_mutex()) == 0);
    } else {
        struct timespec ts;
        assert(clock_gettime(CLOCK_REALTIME, &ts) == 0);
        uint32_t sec = timeout;
        ts.tv_nsec += (timeout - sec) * 1000000000UL;
        ts.tv_sec += sec + ts.tv_nsec / 1000000000UL;
        ts.tv_nsec = ts.tv_nsec % 1000000000UL;
        int ret = pthread_cond_timedwait(cv, get_mutex(), &ts);
        assert(ret == 0 || ret == ETIMEDOUT);
        if (ret == ETIMEDOUT) {
            return false;
        }
    }
    return true;
}
#endif

}

#elif defined(__CHIBIOS__)

#include <ch.h>
#include <osal.h>

namespace msgbus {

typedef condition_variable_t condvar_t;
typedef mutex_t mutex_t;

inline mutex_t *get_mutex() {
    static MUTEX_DECL(mutex);
    return &mutex;
}

inline void _lock()
{
    chMtxLock(get_mutex());
}

inline void _unlock()
{
    chMtxUnlock(get_mutex());
}

inline void condvar_init(condvar_t *cv)
{
    chCondObjectInit(cv);
}

inline void condvar_destroy(condvar_t *cv)
{
    (void)cv;
}

inline void condvar_broadcast(condvar_t *cv)
{
    chCondBroadcast(cv);
}

inline bool condvar_wait(condvar_t *cv, float timeout)
{
    if (timeout == 0) {
        return false;
    }
    systime_t timeout_sys;
    if (timeout < 0) {
        timeout_sys = TIME_INFINITE;
    } else {
        timeout_sys = TIME_US2I(timeout*1e6);
    }
    msg_t ret = chCondWaitTimeout(cv, timeout_sys);
    if (ret == MSG_TIMEOUT) {
        chMtxLock(get_mutex());
        return false;
    } else {
        return true;
    }
}

}

#else // add your platform here
#error undefined platform
#endif

#endif /* MSGBUS_PORT_HPP */
