#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <hal.h>
#include "parameter/parameter.h"
#include "msgbus/msgbus.h"

extern BaseSequentialStream* stdout;

extern parameter_namespace_t parameters;
extern parameter_t board_name;

extern msgbus_t bus;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */