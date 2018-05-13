#ifndef STATISTICS_H
#define STATISTICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t nb_samples;
    float mean;
    float m2;
    float min;
    float max;
} statistics_t;

void statistics_reset(statistics_t *s);
void statistics_add_sample(statistics_t *s, float new_sample);
float statistics_get_min(statistics_t *s);
float statistics_get_max(statistics_t *s);
float statistics_get_mean(statistics_t *s);
float statistics_get_var(statistics_t *s);
float statistics_get_stddev(statistics_t *s);
uint32_t statistics_get_nb_samples(statistics_t *s);

#ifdef __cplusplus
}
#endif

#endif // STATISTICS_H
