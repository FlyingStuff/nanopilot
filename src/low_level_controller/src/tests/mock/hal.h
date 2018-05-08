#ifndef HAL_H
#define HAL_H

#include <stdbool.h>

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef int SDCDriver;
static SDCDriver SDCD1;

static inline
bool sdcConnect(SDCDriver *sdcp) {(void)sdcp; return true;}
static inline
bool sdcDisconnect(SDCDriver *sdcp) {(void)sdcp; return true;}

#ifdef __cplusplus
}
#endif

#endif /* HAL_H */