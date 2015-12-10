#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include "parameter/parameter.h"

extern BaseSequentialStream* stdout;

extern parameter_namespace_t parameters;
extern parameter_t board_name;

void sd_card_activity(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */