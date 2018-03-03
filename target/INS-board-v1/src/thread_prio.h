#ifndef THREAD_PRIO_H
#define THREAD_PRIO_H

// use priorities in the range of 2 to 127, higher number means higher priority

#define THD_PRIO_LED                                    2
#define THD_PRIO_SHELL                                  100
#define THD_PRIO_STREAM                                 4
#define THD_PRIO_DATAGRAM_MSG_HANDLER                   5
#define THD_PRIO_RC_SUMD_IN                             9
#define THD_PRIO_SENSOR_DRV_I2C                         10
#define THD_PRIO_SENSOR_DRV_I2C_BARO                    10
#define THD_PRIO_SENSOR_DRV_SPI                         11
#define THD_PRIO_SENSOR_DRV_I2C_EXT                     10
#define THD_PRIO_SENSOR_ATTITUDE_DETERMINATION          3

#endif /* THREAD_PRIO_H */