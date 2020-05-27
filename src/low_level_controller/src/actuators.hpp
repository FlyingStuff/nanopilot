#ifndef ACTUATORS_HPP
#define ACTUATORS_HPP

#include <board.h>
#include <array>
#include "timestamp.h"

struct actuators_t {
    actuators_t() {
        std::fill(actuators.begin(), actuators.end(), 0);
        actuators_len = 0;
    }
    std::array<float, NB_ACTUATORS> actuators;
    unsigned actuators_len;
};

struct actuators_stamped_t {
    actuators_t actuators;
    timestamp_t timestamp;
};

void actuators_set_output(const actuators_t &out);

extern "C"
void actuators_disable_all(void);

#endif /* ACTUATORS_HPP */
