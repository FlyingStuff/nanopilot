#include "hott/telemetry.h"
#include "types/sensors.h"
#include "types/vario_msg.h"
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
    struct hott_tm_gps_s gps;
    hott_tm_handler_enable_gps(&tm, &gps);
    hott_tm_gps_zero(&gps);

    msgbus_subscriber_t sub_dynamic_pressure;
    msgbus_subscriber_t sub_vario;
    msgbus_topic_subscribe(&sub_dynamic_pressure, _bus, "/sensors/ms4525do", 1000000);
    msgbus_topic_subscribe(&sub_vario, _bus, "/vario", 1000000);

    msgbus_subscriber_assert_type(&sub_dynamic_pressure, &dynamic_pressure_sample_type);
    msgbus_subscriber_assert_type(&sub_vario, &vario_type);
    dynamic_pressure_sample_t dp;
    vario_t vario;

    msgbus_subscriber_wait_for_update(&sub_dynamic_pressure, MSGBUS_TIMEOUT_NEVER);
    msgbus_subscriber_read(&sub_dynamic_pressure, &dp);
    float q0 = dp.dynamic_pressure;

    while (true) {
        char c = streamGet(_tm_uart);

        const float VBATT_DIV_GAIN = 6.5f; // R1: 1.87M R2: 340K
        float v_bat = analog_get_voltage(ANALOG_CH_CONN3_TX) * VBATT_DIV_GAIN;
        gam.voltage = v_bat;
        gam.batt1_volt = v_bat;
        if (v_bat < 3.3f) {
            gam.alarm = HOTT_TM_GAM_ALARM_MIN_CELL_VOLT;
        } else {
            gam.alarm = 0;
        }
        gam.batt2_volt = analog_get_vdc();
        if (msgbus_subscriber_has_update(&sub_dynamic_pressure)) {
            msgbus_subscriber_read(&sub_dynamic_pressure, &dp);
            gam.speed = gps.speed = dynamic_pressure_to_airspeed(dp.dynamic_pressure-q0);
            gam.temp2 = dp.temperature;
            // log_debug("speed %f", (double)gam.speed);
        }
        if (msgbus_subscriber_has_update(&sub_vario)) {
            msgbus_subscriber_read(&sub_vario, &vario);
            gam.height = vario.height;
            gam.climb_rate = vario.climb_rate;
            gps.climb_rate = vario.climb_rate_total_energy_compensated;
            // log_debug("height %f, climb %f, climb tek %f", (double)gam.height, (double)gam.climb_rate, (double)gps.climb_rate);
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
