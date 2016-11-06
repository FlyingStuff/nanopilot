#include "msgbus.h"


void msgbus_init(msgbus_t *bus)
{
    messagebus_init(bus);
}


void msgbus_topic_create(msgbus_topic_t *topic,
                         msgbus_t *bus,
                         const msgbus_type_definition_t *type,
                         void *buffer,
                         const char *name)
{
    messagebus_topic_create(topic, bus, type, buffer, name);
}


msgbus_topic_t *msgbus_find_topic(msgbus_t *bus, const char *name)
{

    return messagebus_find_topic(bus, name);
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
    messagebus_topic_publish(topic, (void*)val);
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
    } else { // todo timeout not working with current implementation
        topic = messagebus_find_topic_blocking(bus, name);
    }
    sub->topic = topic;
    sub->pub_seq_nbr = 0; // TODO: init to topic seq_nbr - 1
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
            messagebus_topic_wait(sub->topic, NULL);
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
    messagebus_topic_read(sub->topic, dest);
}


msgbus_topic_t *msgbus_subscriber_get_topic(msgbus_subscriber_t *sub)
{
    return sub->topic;
}
