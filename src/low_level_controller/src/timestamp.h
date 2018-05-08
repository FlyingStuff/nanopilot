#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>

/* Timestamp
 * monotonic, provides ns resolution, never overflows (every 585 years)
 */
typedef uint64_t timestamp_t;


#ifdef __cplusplus
extern "C" {
#endif


/*
 * Obtain current timestamp
 */
timestamp_t timestamp_get(void);


/*
 * Compute duration between two timestamps
 *  The duration from the first timestamp (t1) to the second (t2) can be
 *  positive or negative. It is positive when t1 was taken before t2.
 */
int64_t timestamp_duration_ns(timestamp_t t1, timestamp_t t2);
float timestamp_duration(timestamp_t t1, timestamp_t t2);


#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_H */
