#ifndef PARAMETER_STORAGE_H
#define PARAMETER_STORAGE_H

#include <parameter/parameter.h>

#ifdef __cplusplus
extern "C" {
#endif

extern parameter_namespace_t parameters;

void parameter_init(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_STORAGE_H */