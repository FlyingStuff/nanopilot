#include "msgbus.h"


void msgbus_init(msgbus_t *bus)
{
    messagebus_condvar_wrapper_init(&bus->sync);
    messagebus_init(&bus->bus, &bus->sync, &bus->sync);
}


void msgbus_topic_create(msgbus_topic_t *topic,
                         msgbus_t *bus,
                         const msgbus_type_definition_t *type,
                         void *buffer,
                         const char *name)
{
    topic->type = type;
    topic->pub_seq_nbr = 0;
    messagebus_condvar_wrapper_init(&topic->sync);
    messagebus_topic_init(&topic->topic, &topic->sync, &topic->sync, buffer, type->struct_size);
    messagebus_advertise_topic(&bus->bus, &topic->topic, name);
}


msgbus_topic_t *msgbus_find_topic(msgbus_t *bus, const char *name)
{
    messagebus_topic_t *wrapped_topic;
    wrapped_topic = messagebus_find_topic(&bus->bus, name);
    if (wrapped_topic == NULL) {
        return NULL;
    }
    return (msgbus_topic_t *) ((void*)wrapped_topic - offsetof(msgbus_topic_t,topic));
}


msgbus_topic_t *msgbus_iterate_topics(msgbus_t *bus)
{
    messagebus_topic_t *wrapped_topic = bus->bus.topics.head;
    if (wrapped_topic == NULL) {
        return NULL;
    }
    return ((void*)wrapped_topic - offsetof(msgbus_topic_t,topic));
}


msgbus_topic_t *msgbus_iterate_topics_next(msgbus_topic_t *topic)
{
    messagebus_topic_t *wrapped_topic = topic->topic.next;
    if (wrapped_topic == NULL) {
        return NULL;
    }
    return ((void*)wrapped_topic - offsetof(msgbus_topic_t,topic));
}


void msgbus_topic_publish(msgbus_topic_t *topic, const void *val)
{
    messagebus_topic_publish(&topic->topic, (void*)val, topic->type->struct_size);
    topic->pub_seq_nbr++;
}


const msgbus_type_definition_t *msgbus_topic_get_type(msgbus_topic_t *topic)
{
    return topic->type;
}


const char *msgbus_topic_get_name(msgbus_topic_t *topic)
{
    return topic->topic.name;
}


bool msgbus_topic_subscribe(msgbus_subscriber_t *sub,
                            msgbus_t *bus,
                            const char *name,
                            uint64_t timeout_us)
{
    msgbus_topic_t *topic;
    if (timeout_us == MSGBUS_TIMEOUT_IMMEDIATE) {
        topic = msgbus_find_topic(bus, name);
    } else { // todo timeout not working with current implementation
        messagebus_topic_t *wrapped_topic;
        wrapped_topic = messagebus_find_topic_blocking(&bus->bus, name);
        if (wrapped_topic == NULL) {
            topic = NULL;
        } else {
            topic = (msgbus_topic_t *) ((void*)wrapped_topic - offsetof(msgbus_topic_t,topic));
        }
    }
    sub->topic = topic;
    sub->pub_seq_nbr = 0;
    if (topic == NULL) {
        return false;
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
            messagebus_topic_wait(&sub->topic->topic, NULL, 0);
            return true;
        }
    }
}


uint32_t msgbus_subscriber_has_update(msgbus_subscriber_t *sub)
{
    // todo does not handle overflow
    if (sub->topic->pub_seq_nbr > sub->pub_seq_nbr) {
        return sub->topic->pub_seq_nbr - sub->pub_seq_nbr;
    }
    return 0;
}


bool msgbus_subscriber_topic_is_valid(msgbus_subscriber_t *sub)
{
    if (sub->topic->pub_seq_nbr > 0) {
        return true;
    }
    return false;
}


void msgbus_subscriber_read(msgbus_subscriber_t *sub, void *dest)
{
    sub->pub_seq_nbr = sub->topic->pub_seq_nbr;
    messagebus_topic_read(&sub->topic->topic, dest, sub->topic->type->struct_size);
}


msgbus_topic_t *msgbus_subscriber_get_topic(msgbus_subscriber_t *sub)
{
    return sub->topic;
}
