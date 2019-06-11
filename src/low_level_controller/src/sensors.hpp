#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <stdint.h>
#include "timestamp.h"
#include "msgbus/msgbus.hpp"

struct quaternion_s{
    float x;
    float y;
    float z;
    float w;
};

typedef struct {
    float rate[3];                  // [rad/s]
    struct quaternion_s accumulated_angle;
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


extern msgbus::Topic<rate_gyro_sample_t> rate_gyro;
extern msgbus::Topic<accelerometer_sample_t> accelerometer;
extern msgbus::Topic<magnetometer_sample_t> magnetometer;


#endif // SENSORS_HPP
