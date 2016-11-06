#include "messagebus_port.h"

#include "../../messagebus.h"

void messagebus_lock_acquire(void *p)
{
    messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
    pthread_mutex_lock(&wrapper->mutex);
}

void messagebus_lock_release(void *p)
{
    messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
    pthread_mutex_unlock(&wrapper->mutex);
}

void messagebus_condvar_broadcast(void *p)
{
    messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
    pthread_cond_broadcast(&wrapper->cond);
}

void messagebus_condvar_wait(void *p)
{
    messagebus_condvar_wrapper_t *wrapper = (messagebus_condvar_wrapper_t *)p;
    pthread_cond_wait(&wrapper->cond, &wrapper->mutex);
}


void messagebus_condvar_wrapper_init(messagebus_condvar_wrapper_t *c)
{
    pthread_mutex_init(&c->mutex, NULL);
    pthread_cond_init(&c->cond, NULL);
}
