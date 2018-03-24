#include "hott/telemetry.h"
#include "types/sensors.h"
#include "thread_prio.h"
#include "log.h"
#include "airdata.h"
#include "analog.h"

#include "hott_tm.h"

static BaseSequentialStream *_tm_uart = NULL;
static msgbus_t *_bus = NULL;

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
    struct hott_tm_gam_s gam;
    hott_tm_handler_enable_gam(&tm, &gam);
    hott_tm_gam_zero(&gam);

    msgbus_subscriber_t sub_baro;
    msgbus_subscriber_t sub_dynamic_pressure;
    msgbus_topic_subscribe(&sub_baro, _bus, "/sensors/ms5611", 1000000);
    msgbus_topic_subscribe(&sub_dynamic_pressure, _bus, "/sensors/ms4525do", 1000000);

    msgbus_subscriber_assert_type(&sub_baro, &barometer_sample_type);
    msgbus_subscriber_assert_type(&sub_dynamic_pressure, &dynamic_pressure_sample_type);
    dynamic_pressure_sample_t dp;
    barometer_sample_t baro;

    msgbus_subscriber_wait_for_update(&sub_baro, MSGBUS_TIMEOUT_NEVER);
    msgbus_subscriber_read(&sub_baro, &baro);
    float h0 = pressure_to_altitude(baro.static_pressure);
    float h_prev = h0;
    uint64_t t_prev = 0;
    msgbus_subscriber_wait_for_update(&sub_dynamic_pressure, MSGBUS_TIMEOUT_NEVER);
    msgbus_subscriber_read(&sub_dynamic_pressure, &dp);
    float q0 = dp.dynamic_pressure;

    while (true) {
        char c = streamGet(_tm_uart);

        const float VBATT_DIV_GAIN = 6.5f; // R1: 1.87M R2: 340K
        float v_bat = analog_get_voltage(ANALOG_CH_CONN3_RX) * VBATT_DIV_GAIN;
        gam.voltage = v_bat;
        gam.batt1_volt = v_bat;
        if (v_bat < 3.3f) {
            gam.alarm = HOTT_TM_GAM_ALARM_MIN_CELL_VOLT;
        }
        gam.batt2_volt = analog_get_vdc();
        if (msgbus_subscriber_has_update(&sub_baro)) {
            msgbus_subscriber_read(&sub_baro, &baro);
            float h = pressure_to_altitude(baro.static_pressure);
            uint64_t t = baro.timestamp_ns;
            gam.temp1 = baro.temperature;
            gam.height = h - h0;
            gam.pressure = baro.static_pressure;
            float cl = (h - h_prev) / (t - t_prev) * 1000000000UL;
            gam.climb_rate = gam.climb_rate*0.9 + cl * 0.1;
            h_prev = h;
            t_prev = t;
            log_debug("temp %f, height %f, climb %f", (double)gam.temp1, (double)gam.height, (double)gam.climb_rate);
        }
        if (msgbus_subscriber_has_update(&sub_dynamic_pressure)) {
            msgbus_subscriber_read(&sub_dynamic_pressure, &dp);
            gam.speed = dynamic_pressure_to_airspeed(dp.dynamic_pressure-q0);
            gam.temp2 = dp.temperature;
            log_debug("speed %f", (double)gam.speed);
        }

        hott_tm_handler_receive(&tm, c);
    }
}


void hott_tm_start(msgbus_t *bus, BaseSequentialStream *uart)
{
    _bus = bus;
    _tm_uart = uart;
    if (uart != NULL) {
        chThdCreateStatic(hott_tm_task_wa, sizeof(hott_tm_task_wa), THD_PRIO_HOTT_TM, hott_tm_task, NULL);
    }
}
