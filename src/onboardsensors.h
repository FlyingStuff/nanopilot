#ifndef ONBOARDSENSORS_H
#define ONBOARDSENSORS_H

#include "sensors.h"
#include <ch.h>

extern rate_gyro_sample_t onboard_mpu6000_gyro_sample;
extern accelerometer_sample_t onboard_mpu6000_acc_sample;
extern float onboard_mpu6000_temp;
extern accelerometer_sample_t onboard_h3lis331dl_acc_sample;
extern magnetometer_sample_t onboard_hmc5883l_mag_sample;
extern barometer_sample_t onboard_ms5511_baro_sample;

#define SENSOR_EVENT_HMC5883L       (1<<0)
#define SENSOR_EVENT_MPU6000        (1<<1)
#define SENSOR_EVENT_H3LIS331DL     (1<<2)
#define SENSOR_EVENT_MS5611         (1<<3)

extern event_source_t sensor_events;

void onboardsensors_declare_parameters(void);
void onboard_sensors_start(void);

#endif // ONBOARDSENSORS_H