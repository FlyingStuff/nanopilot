#include "msgbus_scheduler.h"

void msgbus_scheduler_init(msgbus_scheduler_t *sched,
                           msgbus_t *bus,
                           msgbus_scheduler_task_buffer_space_t *buf,
                           int buf_nb_elements)
{
    sched->bus = bus;
    sched->buffer_size = buf_nb_elements;
    sched->nb_entries = 0;
    sched->subs = (msgbus_subscriber_t **)buf;
    sched->callbacks = (void (**)(void*))&sched->subs[buf_nb_elements];
    sched->args = (void**)&sched->callbacks[buf_nb_elements];
}


bool msgbus_scheduler_add_task(msgbus_scheduler_t *sched,
                               msgbus_subscriber_t *sub,
                               void (*callback)(void*),
                               void *arg)
{
    if (sched->nb_entries < sched->buffer_size) {
        sched->subs[sched->nb_entries] = sub;
        sched->callbacks[sched->nb_entries] = callback;
        sched->args[sched->nb_entries] = arg;
        sched->nb_entries++;
        return true;
    } else {
        return false;
    }
}


void msgbus_scheduler_spin(msgbus_scheduler_t *sched, uint32_t timeout_us)
{
    msgbus_subscriber_wait_for_update_on_any(sched->subs, sched->nb_entries, timeout_us);
    int i;
    for (i=0; i < sched->nb_entries; i++) {
        if (msgbus_subscriber_has_update(sched->subs[i])) {
            sched->callbacks[i](sched->args[i]);
        }
    }
}

msgbus_t *msgbus_scheduler_get_bus(msgbus_scheduler_t *sched)
{
    return sched->bus;
}