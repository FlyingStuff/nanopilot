#ifndef HOTT_TM_H
#define HOTT_TM_H

#include <hal.h>
#include <msgbus/msgbus.h>

#ifdef __cplusplus
extern "C" {
#endif

void hott_tm_start(msgbus_t *bus, BaseSequentialStream *uart);

#ifdef __cplusplus
}
#endif

#endif /* HOTT_TM_H */