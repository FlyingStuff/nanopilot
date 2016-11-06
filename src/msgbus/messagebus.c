#include "messagebus.h"
#include <string.h>



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
    messagebus_condvar_wait(&topic->condvar, MSGBUS_TIMEOUT_NEVER);

    memcpy(buf, topic->buffer, topic->type->struct_size);

    messagebus_lock_release(&topic->lock);
}
