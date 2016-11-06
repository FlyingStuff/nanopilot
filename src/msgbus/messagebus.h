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

#define TOPIC_NAME_MAX_LENGTH 64

typedef struct topic_s {
    void *buffer;
    const msgbus_type_definition_t *type;
    msgbus_mutex_t lock;
    msgbus_cond_t condvar;
    char name[TOPIC_NAME_MAX_LENGTH+1];
    struct topic_s *next;
    bool published;
    uint32_t pub_seq_nbr;
} messagebus_topic_t;

typedef struct {
    struct {
        messagebus_topic_t *head;
    } topics;
    msgbus_mutex_t lock;
    msgbus_cond_t condvar;
} messagebus_t;

#define MESSAGEBUS_TOPIC_FOREACH(_bus, _topic_var_name) \
    for(int __control = -1; __control < 2 ; __control++) \
        if(__control < 0) { \
            messagebus_lock_acquire(&(_bus)->lock); \
        } else if(__control > 0) { \
            messagebus_lock_release(&(_bus)->lock); \
        } else  \
            for (messagebus_topic_t *(_topic_var_name) = (_bus)->topics.head; \
                    topic != NULL; \
                    (_topic_var_name) = (_topic_var_name)->next)

/** Create a new topic
 *
 * @parameter [in] topic The topic object to create.
 * @parameter [in] bus The bus on which the topic will be advertised.
 * @parameter [in] type The type of the published data
 * @parameter [in] buffer The buffer where the topic messages will be stored.
 * @parameter [in] name The topic name, used to refer to it from the rest
 * of the application.
 *
 * @note The topic name will be truncated to TOPIC_NAME_MAX_LENGTH characters.
 */
void messagebus_topic_create(messagebus_topic_t *topic,
                             messagebus_t *bus,
                             const msgbus_type_definition_t *type,
                             void *buffer,
                             const char *name);

/** Initializes a new message bus with no topics.
 *
 * @parameter [in] bus The messagebus to init.
 */
void messagebus_init(messagebus_t *bus);

/** Finds a topic on the bus.
 *
 * @parameter [in] bus The bus to scan.
 * @parameter [in] name The name of the topic to search.
 *
 * @return A pointer to the topic if it is found, NULL otherwise.
 */
messagebus_topic_t *messagebus_find_topic(messagebus_t *bus, const char *name);

/** Waits until a topic is found on the bus.
 *
 * @parameter [in] bus The bus to scan.
 * @parameter [in] name The name of the topic to search.
 */
messagebus_topic_t *messagebus_find_topic_blocking(messagebus_t *bus, const char *name);

/** Publish a topics on the bus.
 *
 * @parameter [in] topic A pointer to the topic to publish.
 * @parameter [in] buf Pointer to a buffer containing the data to publish.
 *
 */
void messagebus_topic_publish(messagebus_topic_t *topic, void *buf);


/** Reads the content of a single topic.
 *
 * @parameter [in] topic A pointer to the topic to read.
 * @parameter [out] buf Pointer where the read data will be stored.
 *
 * @returns true if the topic was published on at least once.
 * @returns false if the topic was never published to
 */
bool messagebus_topic_read(messagebus_topic_t *topic, void *buf);

/** Wait for an update to be published on the topic.
 *
 * @parameter [in] topic A pointer to the topic to read.
 * @parameter [out] buf Pointer where the read data will be stored.
 */
void messagebus_topic_wait(messagebus_topic_t *topic, void *buf);

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
