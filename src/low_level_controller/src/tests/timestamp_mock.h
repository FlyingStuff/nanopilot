#ifndef TIMESTAMP_MOCK_H
#define TIMESTAMP_MOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void timestamp_mock_set_time_raw(uint64_t t);
void timestamp_mock_set_time_s(double t);
void timestamp_mock_inc_time_raw(uint64_t dt);
void timestamp_mock_inc_time_s(double dt);


#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_MOCK_H */