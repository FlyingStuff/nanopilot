#include "mcucom_port_sync.h"

void mcucom_port_mutex_init(mcucom_port_mutex_t *mutex)
{
    chMtxObjectInit(mutex);
}

void mcucom_port_mutex_acquire(mcucom_port_mutex_t *mutex)
{
    chMtxLock(mutex);
}

void mcucom_port_mutex_release(mcucom_port_mutex_t *mutex)
{
    chMtxUnlock(mutex);
}

void mcucom_port_condvar_init(mcucom_port_cond_t *cond)
{
    chCondObjectInit(cond);
}

void mcucom_port_condvar_broadcast(mcucom_port_cond_t *cond)
{
    chCondBroadcast(cond);
}

bool mcucom_port_condvar_wait(mcucom_port_cond_t *cond, mcucom_port_mutex_t *mutex, uint32_t timeout_us)
{
    (void)mutex;

    if (timeout_us == MCUCOM_PORT_TIMEOUT_IMMEDIATE) {
        return false;
    }
    systime_t timeout;
    if (timeout_us == MCUCOM_PORT_TIMEOUT_NEVER) {
        timeout = TIME_INFINITE;
    } else {
        timeout = US2ST(timeout_us);
    }
    msg_t ret = chCondWaitTimeout(cond, timeout);
    if (ret == MSG_TIMEOUT) {
        chMtxLock(mutex);
        return false;
    } else {
        return true;
    }
}
