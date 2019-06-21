#ifndef HOTT_TM_HPP
#define HOTT_TM_HPP

#include <hal.h>
#include <array>
#include "msgbus/msgbus.hpp"
#include "hott/telemetry.h"


void hott_tm_start(BaseSequentialStream *uart);


#endif /* HOTT_TM_HPP */