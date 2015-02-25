#ifndef PARAMETER_PRINT_H
#define PARAMETER_PRINT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "parameter/parameter.h"
#include <ch.h>

void parameter_print(BaseSequentialStream* stream, parameter_namespace_t *ns);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_PRINT_H */