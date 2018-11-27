#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "timestamp.h"


typedef struct {
    float rate[3];                  // [rad/s]
    timestamp_t timestamp;
} rate_gyro_sample_t;


typedef struct {
    float acceleration[3];          // [m/s^2]
    timestamp_t timestamp;
} accelerometer_sample_t;


typedef struct {
    float pressure;                 // [Pa]
    float temperature;              // [deg Celsius]
    timestamp_t timestamp;
} barometer_sample_t;


typedef struct {
    float magnetic_field[3];        // [T]
    timestamp_t timestamp;
} magnetometer_sample_t;


#endif // SENSORS_H
