#ifndef MCUCOM_SYNC_H
#define MCUCOM_SYNC_H

#include <stdint.h>

#define MCUCOM_PORT_TIMEOUT_NEVER UINT32_MAX
#define MCUCOM_PORT_TIMEOUT_IMMEDIATE 0

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize a mutex */
extern void mcucom_port_mutex_init(mcucom_port_mutex_t *mutex);

/** Acquire a reentrant mutex. */
extern void mcucom_port_mutex_acquire(mcucom_port_mutex_t *mutex);

/** Release a mutex previously acquired by mcucom_port_mutex_acquire. */
extern void mcucom_port_mutex_release(mcucom_port_mutex_t *mutex);

/** Initialize a condition variable */
extern void mcucom_port_condvar_init(mcucom_port_cond_t *cond);

/** Signal all tasks waiting on the given condition variable. */
extern void mcucom_port_condvar_broadcast(mcucom_port_cond_t *cond);

/** Wait on the given condition variable.
 * @returns true if the condition was signaled, false on timeout
 */
extern bool mcucom_port_condvar_wait(mcucom_port_cond_t *cond, mcucom_port_mutex_t *mutex, uint32_t timeout_us);

#ifdef __cplusplus
}
#endif

#endif /* MCUCOM_SYNC_H */
