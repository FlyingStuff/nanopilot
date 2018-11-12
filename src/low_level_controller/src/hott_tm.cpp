#include "hott/telemetry.h"
#include "thread_prio.h"
#include "log.h"

#include "hott_tm.hpp"

msgbus::Topic<struct hott_tm_gam_s> TelemetryGAM;
msgbus::Topic<struct hott_tm_gps_s> TelemetryGPS;

static BaseSequentialStream *_tm_uart = NULL;

static void uart_putc(char c)
{
    streamPut(_tm_uart, c);
    streamGet(_tm_uart); // read back
}

static void delay_ms(int ms)
{
    chThdSleepMilliseconds(ms);
}

static THD_WORKING_AREA(hott_tm_task_wa, 2048);
static THD_FUNCTION(hott_tm_task, arg)
{
    (void)arg;
    chRegSetThreadName("hott tm");
    hott_tm_handler_t tm;
    hott_tm_handler_init(&tm, uart_putc, delay_ms);
    auto gam_sub = msgbus::subscribe(TelemetryGAM);
    struct hott_tm_gam_s gam;
    hott_tm_handler_enable_gam(&tm, &gam);
    hott_tm_gam_zero(&gam);
    auto gps_sub = msgbus::subscribe(TelemetryGPS);
    struct hott_tm_gps_s gps;
    hott_tm_handler_enable_gps(&tm, &gps);
    hott_tm_gps_zero(&gps);

    while (true) {
        char c = streamGet(_tm_uart);
        if (gam_sub.has_update()) {
            gam = gam_sub.get_value();
        }
        if (gps_sub.has_update()) {
            gps = gps_sub.get_value();
        }
        hott_tm_handler_receive(&tm, c);
    }
}

void hott_tm_start(BaseSequentialStream *uart)
{
    _tm_uart = uart;
    if (uart != NULL) {
        chThdCreateStatic(hott_tm_task_wa, sizeof(hott_tm_task_wa), THD_PRIO_HOTT_TM, hott_tm_task, NULL);
    }
}
