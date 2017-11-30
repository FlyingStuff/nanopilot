#ifndef MCUCOM_PORT_SYNC_H
#define MCUCOM_PORT_SYNC_H

#include <pthread.h>

typedef pthread_cond_t mcucom_port_cond_t;
typedef pthread_mutex_t mcucom_port_mutex_t;

#include "../mcucom_sync_api.h"

#endif /* MCUCOM_PORT_SYNC_H */
