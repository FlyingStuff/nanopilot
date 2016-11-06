#ifndef MESSAGEBUS_H
#define MESSAGEBUS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <messagebus_port.h>
#include "type_definition.h"
#include "msgbus.h"


#define MESSAGEBUS_TOPIC_FOREACH(_bus, _topic_var_name) \
    for(int __control = -1; __control < 2 ; __control++) \
        if(__control < 0) { \
            messagebus_lock_acquire(&(_bus)->lock); \
        } else if(__control > 0) { \
            messagebus_lock_release(&(_bus)->lock); \
        } else  \
            for (msgbus_topic_t *(_topic_var_name) = (_bus)->topics.head; \
                    topic != NULL; \
                    (_topic_var_name) = (_topic_var_name)->next)


/** Reads the content of a single topic.
 *
 * @parameter [in] topic A pointer to the topic to read.
 * @parameter [out] buf Pointer where the read data will be stored.
 *
 * @returns true if the topic was published on at least once.
 * @returns false if the topic was never published to
 */
bool messagebus_topic_read(msgbus_topic_t *topic, void *buf);

/** Wait for an update to be published on the topic.
 *
 * @parameter [in] topic A pointer to the topic to read.
 * @parameter [out] buf Pointer where the read data will be stored.
 */
void messagebus_topic_wait(msgbus_topic_t *topic, void *buf);

/** @defgroup portable Portable functions, platform specific.
 * @{*/

/** Initialize a mutex */
extern void messagebus_lock_init(msgbus_mutex_t *mutex);

/** Acquire a reentrant mutex. */
extern void messagebus_lock_acquire(msgbus_mutex_t *mutex);

/** Release a mutex previously acquired by messagebus_lock_acquire. */
extern void messagebus_lock_release(msgbus_mutex_t *mutex);

/** Initialize a condition variable */
extern void messagebus_condvar_init(msgbus_cond_t *cond);

/** Signal all tasks waiting on the given condition variable. */
extern void messagebus_condvar_broadcast(msgbus_cond_t *cond);

/** Wait on the given condition variable. */
extern void messagebus_condvar_wait(msgbus_cond_t *cond);

/** @} */

#ifdef __cplusplus
}
#endif
#endif
