#include "timestamp.h"

int64_t timestamp_duration_ns(timestamp_t t1, timestamp_t t2)
{
    return (t2 - t1);
}

float timestamp_duration(timestamp_t t1, timestamp_t t2)
{
    return timestamp_duration_ns(t1, t2) * 1e-9f;
}
