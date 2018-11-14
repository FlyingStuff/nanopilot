#ifndef ACTUATORS_HPP
#define ACTUATORS_HPP

#include <array>
#include "parameter/parameter.h"

const int NB_ACTUATORS = 4;

void initialize_actuators(parameter_namespace_t *ns);
void actuators_set_output(const std::array<float, NB_ACTUATORS> out);

extern "C"
void actuators_disable_all(void);


#endif /* ACTUATORS_HPP */
