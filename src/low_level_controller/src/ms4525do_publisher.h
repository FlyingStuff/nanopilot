#ifndef MS4525DO_PUBLISHER_H
#define MS4525DO_PUBLISHER_H

#include <hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void ms4525do_publisher_start(I2CDriver *i2c, const char *topic);

#ifdef __cplusplus
}
#endif

#endif /* MS4525DO_PUBLISHER_H */