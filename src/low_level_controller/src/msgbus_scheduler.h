#ifndef MSGBUS_SCHEDULER_H
#define MSGBUS_SCHEDULER_H

#include <msgbus/msgbus.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    msgbus_subscriber_t **subs;
    void (**callbacks)(void*);
    void **args;
    int nb_entries;
    int buffer_size;
    msgbus_t *bus;
} msgbus_scheduler_t;

typedef struct {
    msgbus_subscriber_t *sub;
    void (*callback)(void*);
    void *arg;
} msgbus_scheduler_task_buffer_space_t;

void msgbus_scheduler_init(msgbus_scheduler_t *sched,
                           msgbus_t *bus,
                           msgbus_scheduler_task_buffer_space_t *buf,
                           int buf_nb_elements);
msgbus_t *msgbus_scheduler_get_bus(msgbus_scheduler_t *sched);
bool msgbus_scheduler_add_task(msgbus_scheduler_t *sched,
                               msgbus_subscriber_t *sub,
                               void (*callback)(void*),
                               void *arg);
void msgbus_scheduler_spin(msgbus_scheduler_t *sched, uint32_t timeout_us);

int msgbus_scheduler_get_nb_tasks(msgbus_scheduler_t *sched);

#ifdef __cplusplus
}
#endif

#endif /* MSGBUS_SCHEDULER_H */
