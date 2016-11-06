#include "messagebus.h"
#include <string.h>

static void _messagebus_advertise_topic(msgbus_t *bus, msgbus_topic_t *topic, const char *name);


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

void messagebus_init(msgbus_t *bus)
{
    memset(bus, 0, sizeof(msgbus_t));
    messagebus_condvar_init(&bus->condvar);
    messagebus_lock_init(&bus->lock);
}


void messagebus_topic_create(msgbus_topic_t *topic,
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

    _messagebus_advertise_topic(bus, topic, name);
}


static void _messagebus_advertise_topic(msgbus_t *bus, msgbus_topic_t *topic, const char *name)
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

msgbus_topic_t *messagebus_find_topic(msgbus_t *bus, const char *name)
{
    msgbus_topic_t *res;

    messagebus_lock_acquire(&bus->lock);

    res = topic_by_name(bus, name);

    messagebus_lock_release(&bus->lock);

    return res;
}

msgbus_topic_t *messagebus_find_topic_blocking(msgbus_t *bus, const char *name)
{
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

void messagebus_topic_publish(msgbus_topic_t *topic, void *buf)
{
    messagebus_lock_acquire(&topic->lock);

    memcpy(topic->buffer, buf, topic->type->struct_size);
    topic->published = true;
    topic->pub_seq_nbr++;
    messagebus_condvar_broadcast(&topic->condvar);

    messagebus_lock_release(&topic->lock);
}

bool messagebus_topic_read(msgbus_topic_t *topic, void *buf)
{
    bool success = false;
    messagebus_lock_acquire(&topic->lock);

    if (topic->published) {
        success = true;
        memcpy(buf, topic->buffer, topic->type->struct_size);
    }

    messagebus_lock_release(&topic->lock);

    return success;
}

void messagebus_topic_wait(msgbus_topic_t *topic, void *buf)
{
    messagebus_lock_acquire(&topic->lock);
    messagebus_condvar_wait(&topic->condvar);

    memcpy(buf, topic->buffer, topic->type->struct_size);

    messagebus_lock_release(&topic->lock);
}
