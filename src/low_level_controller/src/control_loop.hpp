#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

#include "msgbus/msgbus.hpp"

extern msgbus::Topic<bool> output_armed;

void control_init();
void control_start();

#endif /* CONTROL_LOOP_HPP */
