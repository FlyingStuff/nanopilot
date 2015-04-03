#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "timestamp/timestamp.h"

typedef struct {
    const char *device;
    float full_scale_range;
    float update_rate;
} rate_gyro_t;

typedef struct {
    const rate_gyro_t *sensor;
    float rate[3];                  // [rad/s]
    timestamp_t timestamp;
} rate_gyro_sample_t;


typedef struct {
    const char *device;
    float full_scale_range;
    float update_rate;
    float position[3];
} accelerometer_t;

typedef struct {
    const accelerometer_t *sensor;
    float acceleration[3];          // [m/s^2]
    timestamp_t timestamp;
} accelerometer_sample_t;


typedef struct {
    float pressure;                 // [Pa]
    float temperature;              // [deg Celsius]
    timestamp_t timestamp;
} barometer_sample_t;


typedef struct {
    const char *device;
    float full_scale_range;
    float update_rate;
} magnetometer_t;

typedef struct {
    const magnetometer_t sensor;
    float magnetic_field[3];        // [T]
    timestamp_t timestamp;
} magnetometer_sample_t;


#endif // SENSORS_H
