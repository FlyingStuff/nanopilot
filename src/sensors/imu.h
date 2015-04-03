#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "timestamp/timestamp.h"

typedef struct {
    const char *device;                 // sensor name
    float full_scale_range[3];          // [rad/s]
    float noise_stddev[3];              // [rad/s]
    float update_rate;                  // [Hz]
} rate_gyro_t;

typedef struct {
    const rate_gyro_t *sensor;
    float rate[3];
    timestamp_t timestamp;
} rate_gyro_sample_t;


typedef struct {
    const char *device;                 // sensor name
    float full_scale_range[3];          // [m/s^2]
    float noise_stddev[3];              // [m/s^2]
    float update_rate;                  // [Hz]
    float position[3];                  // [m]
} accelerometer_t;

typedef struct {
    const accelerometer_t *sensor;
    float acceleration[3];
    timestamp_t timestamp;
} accelerometer_sample_t;

#endif // IMU_H
