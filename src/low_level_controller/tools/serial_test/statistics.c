#include <math.h>
#include "statistics.h"

void statistics_reset(statistics_t *s)
{
    s->nb_samples = 0;
    s->mean = 0;
    s->m2 = 0;
    s->min = INFINITY;
    s->max = -INFINITY;
}

void statistics_add_sample(statistics_t *s, float new_sample)
{
    if (new_sample > s->max) {
        s->max = new_sample;
    }
    if (new_sample < s->min) {
        s->min = new_sample;
    }

    // see http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
    s->nb_samples += 1;
    float delta = new_sample - s->mean;
    s->mean += delta / s->nb_samples;
    s->m2 += delta * (new_sample - s->mean);
}

float statistics_get_min(statistics_t *s)
{
    return s->min;
}

float statistics_get_max(statistics_t *s)
{
    return s->max;
}

float statistics_get_mean(statistics_t *s)
{
    return s->mean;
}

float statistics_get_var(statistics_t *s)
{
    if (s->nb_samples < 2) {
        return 0;
    }
    return s->m2 / (s->nb_samples - 1);
}

float statistics_get_stddev(statistics_t *s)
{
    return sqrtf(statistics_get_var(s));
}

uint32_t statistics_get_nb_samples(statistics_t *s)
{
    return s->nb_samples;
}

