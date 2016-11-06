#ifndef TYPED_MESSAGEBUS_H
#define TYPED_MESSAGEBUS_H

#include <messagebus_port.h>
#include "messagebus.h"
#include "type_definition.h"

typedef struct {
    messagebus_topic_t msgbus_topic;
    messagebus_condvar_wrapper_t condvar;
    const messagebus_type_definition_t *type;
} typed_messagebus_topic_t;

typedef struct {
    messagebus_t bus;
    messagebus_condvar_wrapper_t condvar;
} typed_messagebus_t;

#ifdef __cplusplus
extern "C" {
#endif

void typed_messagebus_init(typed_messagebus_t *bus);
void typed_messagebus_topic_init(typed_messagebus_topic_t *topic, const messagebus_type_definition_t *type, void *buffer);
void typed_messagebus_advertise_topic(typed_messagebus_t *bus, typed_messagebus_topic_t *topic, const char *name);
typed_messagebus_topic_t *typed_messagebus_find_topic(typed_messagebus_t *bus, const char *name);
typed_messagebus_topic_t *typed_messagebus_find_topic_blocking(typed_messagebus_t *bus, const char *name);
bool typed_messagebus_topic_publish(typed_messagebus_topic_t *topic, void *buf);
bool typed_messagebus_topic_read(typed_messagebus_topic_t *topic, void *buf);
void typed_messagebus_topic_wait(typed_messagebus_topic_t *topic, void *buf);

#ifdef __cplusplus
}
#endif

#endif /* TYPED_MESSAGEBUS_H */
