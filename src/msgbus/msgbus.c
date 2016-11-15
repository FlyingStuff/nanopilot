#include <string.h>
#include "msgbus.h"


void msgbus_init(msgbus_t *bus)
{
    memset(bus, 0, sizeof(msgbus_t));
    msgbus_condvar_init(&bus->condvar);
    msgbus_mutex_init(&bus->lock);
    msgbus_mutex_init(&bus->topic_update_lock);
}


static msgbus_topic_t *topic_by_name(msgbus_t *bus, const char *name)
{
    msgbus_topic_t *t;
    for (t=bus->topics.head; t!=NULL; t=t->next) {
        if (!strcmp(name, t->name)) {
            return t;
        }
    }

    return NULL;
}


static void advertise_topic(msgbus_t *bus, msgbus_topic_t *topic)
{
    msgbus_mutex_acquire(&bus->lock);

    if (bus->topics.head != NULL) {
        topic->next = bus->topics.head;
    }
    bus->topics.head = topic;

    msgbus_condvar_broadcast(&bus->condvar);

    msgbus_mutex_release(&bus->lock);
}

void msgbus_topic_create(msgbus_topic_t *topic,
                         msgbus_t *bus,
                         const msgbus_type_definition_t *type,
                         void *buffer,
                         const char *name)
{
    memset(topic, 0, sizeof(msgbus_topic_t));
    topic->buffer = buffer;
    topic->type = type;
    topic->name = name;
    topic->bus = bus;
    msgbus_condvar_init(&topic->condvar);

    advertise_topic(bus, topic);
}


msgbus_topic_t *msgbus_find_topic(msgbus_t *bus,
                                  const char *name,
                                  uint32_t timeout_us)
{
    msgbus_topic_t *res = NULL;

    msgbus_mutex_acquire(&bus->lock);

    while (true) {
        res = topic_by_name(bus, name);

        if (res == NULL && timeout_us != MSGBUS_TIMEOUT_IMMEDIATE) {
            msgbus_condvar_wait(&bus->condvar, &bus->lock, timeout_us); // todo handle return value
        } else {
            break;
        }
    }

    msgbus_mutex_release(&bus->lock);

    return res;
}


msgbus_topic_t *msgbus_iterate_topics(msgbus_t *bus)
{
    msgbus_topic_t *t;

    msgbus_mutex_acquire(&bus->lock);

    t = bus->topics.head;

    msgbus_mutex_release(&bus->lock);

    return t;
}


msgbus_topic_t *msgbus_iterate_topics_next(msgbus_topic_t *topic)
{
    return topic->next; // next pointer is const
}


void msgbus_topic_publish(msgbus_topic_t *topic, const void *val)
{
    msgbus_mutex_acquire(&topic->bus->topic_update_lock);

    memcpy(topic->buffer, val, topic->type->struct_size);
    topic->published = true;
    topic->pub_seq_nbr++;
    msgbus_condvar_broadcast(&topic->condvar);

    msgbus_mutex_release(&topic->bus->topic_update_lock);
}


const msgbus_type_definition_t *msgbus_topic_get_type(msgbus_topic_t *topic)
{
    return topic->type;
}


const char *msgbus_topic_get_name(msgbus_topic_t *topic)
{
    return topic->name;
}


bool msgbus_topic_subscribe(msgbus_subscriber_t *sub,
                            msgbus_t *bus,
                            const char *name,
                            uint32_t timeout_us)
{
    msgbus_topic_t *topic = msgbus_find_topic(bus, name, timeout_us);
    sub->topic = topic;
    if (topic == NULL) {
        return false;
    }
    if (topic->published) {
        sub->pub_seq_nbr = topic->pub_seq_nbr - 1;
    } else {
        sub->pub_seq_nbr = 0;
    }
    return true;
}


static uint32_t subscriber_get_nb_updates_with_lock(msgbus_subscriber_t *sub)
{
    return sub->topic->pub_seq_nbr - sub->pub_seq_nbr;
}


bool msgbus_subscriber_wait_for_update(msgbus_subscriber_t *sub,
                                       uint32_t timeout_us)
{
    msgbus_mutex_acquire(&sub->topic->bus->topic_update_lock);

    int updates = subscriber_get_nb_updates_with_lock(sub);
    if (updates == 0 && timeout_us != MSGBUS_TIMEOUT_IMMEDIATE) {
        msgbus_condvar_wait(&sub->topic->condvar, &sub->topic->bus->topic_update_lock, timeout_us);
        updates = subscriber_get_nb_updates_with_lock(sub);
    }

    msgbus_mutex_release(&sub->topic->bus->topic_update_lock);

    if (updates > 0) {
        return true;
    } else {
        return false;
    }
}


uint32_t msgbus_subscriber_has_update(msgbus_subscriber_t *sub)
{
    uint32_t ret;

    msgbus_mutex_acquire(&sub->topic->bus->topic_update_lock);

    ret = subscriber_get_nb_updates_with_lock(sub);

    msgbus_mutex_release(&sub->topic->bus->topic_update_lock);

    return ret;
}


bool msgbus_subscriber_topic_is_valid(msgbus_subscriber_t *sub)
{
    bool is_valid;

    msgbus_mutex_acquire(&sub->topic->bus->topic_update_lock);

    is_valid = sub->topic->published;

    msgbus_mutex_release(&sub->topic->bus->topic_update_lock);

    return is_valid;
}


uint32_t msgbus_subscriber_read(msgbus_subscriber_t *sub, void *dest)
{
    uint32_t ret;

    msgbus_mutex_acquire(&sub->topic->bus->topic_update_lock);

    ret = subscriber_get_nb_updates_with_lock(sub);
    sub->pub_seq_nbr = sub->topic->pub_seq_nbr;
    memcpy(dest, sub->topic->buffer, sub->topic->type->struct_size);

    msgbus_mutex_release(&sub->topic->bus->topic_update_lock);

    return ret;
}


msgbus_topic_t *msgbus_subscriber_get_topic(msgbus_subscriber_t *sub)
{
    return sub->topic;
}
