#ifndef ONBOARDSENSORS_H
#define ONBOARDSENSORS_H

#include "parameter/parameter.h"
#include <ch.h>

#ifdef __cplusplus
extern "C" {
#endif


void onboardsensors_declare_parameters(parameter_namespace_t *ns);
void onboard_sensors_start(void);


#ifdef __cplusplus
}
#endif

#endif // ONBOARDSENSORS_H