#include <math.h>
#include "airdata.h"

float pressure_to_altitude(float pressure)
{
    /* "A Quick Derivation relating altitude to air pressure"
     * 2004 Portland State Aerospace Society
     */
    return 44330.8f - 4946.54f * powf(pressure, 0.1902632f);
}

#define AIR_DENSITY 1.225f

float dynamic_pressure_to_airspeed(float q)
{
    return sqrtf(2*fabsf(q) / AIR_DENSITY);
}
