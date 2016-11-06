#include "type_definition.h"
#include "typed_messagebus.h"

void typed_messagebus_init(typed_messagebus_t *bus)
{
    messagebus_condvar_wrapper_init(&bus->condvar);
    messagebus_init(&bus->bus, &bus->condvar, &bus->condvar);
}

void typed_messagebus_topic_init(typed_messagebus_topic_t *topic, const messagebus_type_definition_t *type, void *buffer)
{
    topic->type = type;
    messagebus_condvar_wrapper_init(&topic->condvar);
    messagebus_topic_init(&topic->msgbus_topic, &topic->condvar, &topic->condvar, buffer, type->struct_size);
}

void typed_messagebus_advertise_topic(typed_messagebus_t *bus, typed_messagebus_topic_t *topic, const char *name)
{
    messagebus_advertise_topic(&bus->bus, &topic->msgbus_topic, name);
}

typed_messagebus_topic_t *typed_messagebus_find_topic(typed_messagebus_t *bus, const char *name)
{
    messagebus_topic_t *t = messagebus_find_topic(&bus->bus, name);
    return (typed_messagebus_topic_t *)((char *)t + offsetof(typed_messagebus_topic_t, msgbus_topic));
}

typed_messagebus_topic_t *typed_messagebus_find_topic_blocking(typed_messagebus_t *bus, const char *name)
{
    messagebus_topic_t *t = messagebus_find_topic_blocking(&bus->bus, name);
    return (typed_messagebus_topic_t *)((char *)t + offsetof(typed_messagebus_topic_t, msgbus_topic));
}

bool typed_messagebus_topic_publish(typed_messagebus_topic_t *topic, void *buf)
{
    return messagebus_topic_publish(&topic->msgbus_topic, buf, topic->type->struct_size);
}

bool typed_messagebus_topic_read(typed_messagebus_topic_t *topic, void *buf)
{
    return messagebus_topic_read(&topic->msgbus_topic, buf, topic->type->struct_size);
}

void typed_messagebus_topic_wait(typed_messagebus_topic_t *topic, void *buf)
{
    return messagebus_topic_wait(&topic->msgbus_topic, buf, topic->type->struct_size);
}
