#include <timestamp/timestamp.h>

#define TIMESTAMP_RES 1000000 // timestamp is in us

static uint64_t current_time = 0;

void timestamp_mock_set_time_raw(uint64_t t)
{
    current_time = t;
}

void timestamp_mock_set_time_s(double t)
{
    current_time = t*TIMESTAMP_RES;
}

void timestamp_mock_inc_time_raw(uint64_t dt)
{
    current_time += dt;
}

void timestamp_mock_inc_time_s(double dt)
{
    current_time += dt*TIMESTAMP_RES;
}

timestamp_t timestamp_get(void)
{
    return current_time;
}
