#ifndef ONBOARDSENSORS_H
#define ONBOARDSENSORS_H

#include "sensors.h"
#include "parameter/parameter.h"
#include <ch.h>

#ifdef __cplusplus
extern "C" {
#endif


void onboard_sensor_get_mpu6000_gyro_sample(rate_gyro_sample_t *out);
void onboard_sensor_get_mpu6000_acc_sample(accelerometer_sample_t *out);
float onboard_sensor_get_mpu6000_temp(void);
void onboard_sensor_get_h3lis331dl_acc_sample(accelerometer_sample_t *out);
void onboard_sensor_get_hmc5883l_mag_sample(magnetometer_sample_t *out);
void onboard_sensor_get_ms5511_baro_sample(barometer_sample_t *out);


#define SENSOR_EVENT_HMC5883L       (1<<0)
#define SENSOR_EVENT_MPU6000        (1<<1)
#define SENSOR_EVENT_H3LIS331DL     (1<<2)
#define SENSOR_EVENT_MS5611         (1<<3)

extern event_source_t sensor_events;

void onboardsensors_declare_parameters(parameter_namespace_t *ns);
void onboard_sensors_start(void);


#ifdef __cplusplus
}
#endif

#endif // ONBOARDSENSORS_H