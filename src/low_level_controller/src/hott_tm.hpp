#ifndef HOTT_TM_HPP
#define HOTT_TM_HPP

#include <hal.h>
#include <array>
#include "msgbus/msgbus.hpp"
#include "hott/telemetry.h"

extern msgbus::Topic<struct hott_tm_gam_s> TelemetryGAM;
extern msgbus::Topic<struct hott_tm_gps_s> TelemetryGPS;

void hott_tm_start(BaseSequentialStream *uart);


#endif /* HOTT_TM_HPP */