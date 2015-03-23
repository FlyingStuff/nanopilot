#ifndef ONBOARDSENSORS_H
#define ONBOARDSENSORS_H

#include "imu.h"
#include <ch.h>

extern rate_gyro_sample_t mpu_gyro_sample;
extern accelerometer_sample_t mpu_acc_sample;
extern float mpu_temp;

#define SENSOR_EVENT_HMC5883L       1
#define SENSOR_EVENT_MPU6000        2
#define SENSOR_EVENT_H3LIS331DL     4

extern event_source_t sensor_events;

void onboardsensors_declare_parameters(void);
void onboard_sensors_start(void);

#endif // ONBOARDSENSORS_H