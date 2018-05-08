#ifndef AIRDATA_H
#define AIRDATA_H

#ifdef __cplusplus
extern "C" {
#endif

float pressure_to_altitude(float pressure);
float dynamic_pressure_to_airspeed(float q);


#ifdef __cplusplus
}
#endif

#endif /* AIRDATA_H */