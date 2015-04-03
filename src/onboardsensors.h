#ifndef ONBOARDSENSORS_H
#define ONBOARDSENSORS_H

#include "imu.h"
#include <ch.h>

extern rate_gyro_sample_t mpu_gyro_sample;
extern accelerometer_sample_t mpu_acc_sample;
extern float mpu_temp;
extern accelerometer_sample_t h3lis331dl_acc_sample;
extern float magnetic_field[3]; // [gauss]
extern float static_pressure; // [Pa]
extern float air_temp; // [deg Celsius]

#define SENSOR_EVENT_HMC5883L       (1<<0)
#define SENSOR_EVENT_MPU6000        (1<<1)
#define SENSOR_EVENT_H3LIS331DL     (1<<2)
#define SENSOR_EVENT_MS5611         (1<<3)

extern event_source_t sensor_events;

void onboardsensors_declare_parameters(void);
void onboard_sensors_start(void);

#endif // ONBOARDSENSORS_H