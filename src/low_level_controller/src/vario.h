#ifndef VARIO_H
#define VARIO_H

#include <msgbus/msgbus.h>
#include "msgbus_scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

void vario_init(msgbus_t *bus);
void vario_register_tasks(msgbus_scheduler_t *sched);

#ifdef __cplusplus
}
#endif

#endif /* VARIO_H */
