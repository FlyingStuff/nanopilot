#include <string.h>
#include "messagebus.h"
#include "msgbus.h"


void msgbus_init(msgbus_t *bus)
{
    memset(bus, 0, sizeof(msgbus_t));
    messagebus_condvar_init(&bus->condvar);
    messagebus_lock_init(&bus->lock);
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


static void advertise_topic(msgbus_t *bus, msgbus_topic_t *topic, const char *name)
{
    memset(topic->name, 0, sizeof(topic->name));
    strncpy(topic->name, name, TOPIC_NAME_MAX_LENGTH);

    messagebus_lock_acquire(&bus->lock);

    if (bus->topics.head != NULL) {
        topic->next = bus->topics.head;
    }
    bus->topics.head = topic;

    messagebus_condvar_broadcast(&bus->condvar);

    messagebus_lock_release(&bus->lock);
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
    messagebus_condvar_init(&topic->condvar);
    messagebus_lock_init(&topic->lock);

    advertise_topic(bus, topic, name);
}


msgbus_topic_t *msgbus_find_topic(msgbus_t *bus, const char *name)
{
    msgbus_topic_t *res;

    messagebus_lock_acquire(&bus->lock);

    res = topic_by_name(bus, name);

    messagebus_lock_release(&bus->lock);

    return res;
}

msgbus_topic_t *msgbus_find_topic_blocking(msgbus_t *bus,
                                           const char *name,
                                           uint64_t timeout_us)
{
    // todo timeout ignored
    msgbus_topic_t *res = NULL;

    messagebus_lock_acquire(&bus->lock);

    while (res == NULL) {
        res = topic_by_name(bus, name);

        if (res == NULL) {
            messagebus_condvar_wait(&bus->condvar);
        }
    }

    messagebus_lock_release(&bus->lock);

    return res;
}


msgbus_topic_t *msgbus_iterate_topics(msgbus_t *bus)
{
    return bus->topics.head;
}


msgbus_topic_t *msgbus_iterate_topics_next(msgbus_topic_t *topic)
{
    return topic->next;
}


void msgbus_topic_publish(msgbus_topic_t *topic, const void *val)
{
    messagebus_lock_acquire(&topic->lock);

    memcpy(topic->buffer, val, topic->type->struct_size);
    topic->published = true;
    topic->pub_seq_nbr++;
    messagebus_condvar_broadcast(&topic->condvar);

    messagebus_lock_release(&topic->lock);
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
                            uint64_t timeout_us)
{
    msgbus_topic_t *topic;
    if (timeout_us == MSGBUS_TIMEOUT_IMMEDIATE) {
        topic = msgbus_find_topic(bus, name);
    } else {
        topic = msgbus_find_topic_blocking(bus, name, MSGBUS_TIMEOUT_IMMEDIATE);
    }
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


bool msgbus_subscriber_wait_for_update(msgbus_subscriber_t *sub,
                                       uint64_t timeout_us)
{
    int updates = msgbus_subscriber_has_update(sub);
    if (updates > 0) {
        return true;
    } else {
        if (timeout_us == MSGBUS_TIMEOUT_IMMEDIATE) {
            return false;
        } else { // todo timeout not working with current implementation
            messagebus_topic_wait(sub->topic, NULL);
            return true;
        }
    }
}


uint32_t msgbus_subscriber_has_update(msgbus_subscriber_t *sub)
{
    uint32_t ret;

    messagebus_lock_acquire(&sub->topic->lock);

    ret = sub->topic->pub_seq_nbr - sub->pub_seq_nbr;

    messagebus_lock_release(&sub->topic->lock);

    return ret;
}


bool msgbus_subscriber_topic_is_valid(msgbus_subscriber_t *sub)
{
    bool is_valid;

    // lock
    is_valid = sub->topic->published;
    // unlock

    return is_valid;
}


void msgbus_subscriber_read(msgbus_subscriber_t *sub, void *dest)
{
    sub->pub_seq_nbr = sub->topic->pub_seq_nbr;
    messagebus_topic_read(sub->topic, dest);
}


msgbus_topic_t *msgbus_subscriber_get_topic(msgbus_subscriber_t *sub)
{
    return sub->topic;
}
