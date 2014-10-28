#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#define SENSOR_HEALTH_OK                0
#define SENSOR_HEALTH_SELF_TEST_FAILED  1
#define SENSOR_HEALTH_FLAG_MALFUNCTION  2

typedef struct {
    const char *device;                 // sensor name
    float full_scale_range[3];          // rad/s
    float noise_stddev[3];              // rad/s
    float update_rate;                  // Hz
    int health;                         // health status
    // linear_calibration_t calibration;
    // sensor_orientation_t *orientation;
} rate_gyro_t;

typedef struct {
    const char *device;                 // sensor name
    float full_scale_range[3];          // m/s^2
    float noise_stddev[3];              // m/s^2
    float update_rate;                  // Hz
    int health;                         // health status
    // linear_calibration_t calibration;
    // sensor_position_t *position;
    // sensor_orientation_t *orientation;
} accelerometer_t;

typedef struct {
    rate_gyro_t *sensor;
    float rate[3];
    // timestamp_t timestamp;
} rate_gyro_sample_t;

typedef struct {
    accelerometer_t *sensor;
    float acceleration[3];
    // timestamp_t timestamp;
} accelerometer_sample_t;

#endif // IMU_H
