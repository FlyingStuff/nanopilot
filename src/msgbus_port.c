#include <ch.h>
#include "msgbus_port.h"

// void messagebus_lock_acquire(void *p)
// {
//     messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
//     chMtxLock(&wrapper->lock);
// }

// void messagebus_lock_release(void *p)
// {
//     messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
//     chMtxUnlock(&wrapper->lock);
// }

// void messagebus_condvar_broadcast(void *p)
// {
//     messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
//     chCondBroadcast(&wrapper->cond);
// }

// void messagebus_condvar_wait(void *p)
// {
//     messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
//     chCondWait(&wrapper->cond);
// }

// void messagebus_condvar_wrapper_init(messagebus_condvar_wrapper_t *c)
// {
//     chMtxObjectInit(&c->lock);
//     chCondObjectInit(&c->cond);
// }

void msgbus_mutex_init(msgbus_mutex_t *mutex)
{
    (void)mutex;
}

void msgbus_mutex_acquire(msgbus_mutex_t *mutex)
{
    (void)mutex;
}

void msgbus_mutex_release(msgbus_mutex_t *mutex)
{
    (void)mutex;
}

void msgbus_condvar_init(msgbus_cond_t *cond)
{
    (void)cond;
}

void msgbus_condvar_broadcast(msgbus_cond_t *cond)
{
    (void)cond;
}

void msgbus_condvar_wait(msgbus_cond_t *cond, msgbus_mutex_t *mutex, uint32_t timeout_us)
{
    (void)cond;
    (void)mutex;
    (void)timeout_us;
}
