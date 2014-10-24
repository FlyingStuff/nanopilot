#ifndef ONBOARDSENSORS_H
#define ONBOARDSENSORS_H

extern float gyro[3];
extern float acc[3];
extern float temp;

void onboard_sensors_start(void);

#endif // ONBOARDSENSORS_H