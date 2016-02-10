#ifndef EXTI_H
#define EXTI_H

#include <ch.h>

#define EXTI_EVENT_HMC5883L_DRDY   1
#define EXTI_EVENT_MPU6000_INT     2
#define EXTI_EVENT_H3LIS331DL_INT  4

extern event_source_t exti_events;

void exti_setup(void);

#endif // EXTI_H
