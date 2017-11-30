#ifndef MCUCOM_PORT_SYNC_H
#define MCUCOM_PORT_SYNC_H

typedef int mcucom_port_cond_t;
typedef int mcucom_port_mutex_t;

#include "../mcucom_sync_api.h"


#ifdef __cplusplus
// this is the unit test api

#include <functional>

void synchronization_init_mocks_enable(bool enabled);
void lock_mocks_enable(bool enabled);
void condvar_mocks_enable(bool enabled);
void condvar_mocks_ignore_cv_pointer_arg(bool enabled);

void set_condvar_wait_side_effect(std::function<void()> side_effect);
void clear_condvar_wait_side_effect();

#endif /* __cplusplus */


#endif /* MCUCOM_PORT_SYNC_H */
