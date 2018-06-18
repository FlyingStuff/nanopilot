#include "vario.h"
#include "types/sensors.h"
#include "types/vario_msg.h"
#include "airdata.h"
#include "timestamp.h"
#include "log.h"

static msgbus_subscriber_t _static_pressure_sub;
static msgbus_subscriber_t _dynamic_pressure_sub;

static msgbus_topic_t _vario;
static vario_t _vario_buffer;


#define CLIMB_RATE_FILTER_CONSTANT 0.05f
static float filtered_climb_rate = 0;
#define KINETIC_ENERGY_FILTER_CONSTANT 0.02f
static float filtered_kinetic_energy_equivalent_climb_rate = 0;

#define NB_ZERO_SAMPLES_HEIGHT 10
static int zero_initial_height_samples = 0;
static float zero_altitude = 0;
static float prev_height = 0;
static uint64_t prev_height_timestamp;

void static_pressure_update(void *arg)
{
    (void)arg;
    barometer_sample_t baro;
    msgbus_subscriber_read(&_static_pressure_sub, &baro);
    float altitude = pressure_to_altitude(baro.static_pressure);
    if (zero_initial_height_samples < NB_ZERO_SAMPLES_HEIGHT) {
        zero_initial_height_samples++;
        zero_altitude += altitude / NB_ZERO_SAMPLES_HEIGHT;
    } else {
        float height = altitude - zero_altitude;
        float dt = timestamp_duration(prev_height_timestamp, baro.timestamp_ns);
        float climb_rate = (height - prev_height) / dt;
        filtered_climb_rate *= (1-CLIMB_RATE_FILTER_CONSTANT);
        filtered_climb_rate += CLIMB_RATE_FILTER_CONSTANT * climb_rate;
        prev_height = height;

        // log_debug("height %f, climb %f, dt %f", (double)height, (double)climb_rate, (double)dt);

        vario_t vario_msg;
        vario_msg.height = height;
        vario_msg.climb_rate = filtered_climb_rate;
        vario_msg.climb_rate_total_energy_compensated = filtered_climb_rate + filtered_kinetic_energy_equivalent_climb_rate;
        vario_msg.timestamp_ns = baro.timestamp_ns;
        msgbus_topic_publish(&_vario, &vario_msg);
    }
    prev_height_timestamp = baro.timestamp_ns;
}


#define NB_ZERO_SAMPLES_DYNAMIC_PRESSURE 10
static int zero_dynamic_pressure_samples = 0;
static float zero_dynamic_pressure_offset = 0;
static float prev_kinetic_energy = 0;
static uint64_t prev_kinetic_energy_timestamp;


void dynamic_pressure_update(void *arg)
{
    (void)arg;
    dynamic_pressure_sample_t dp;
    msgbus_subscriber_read(&_dynamic_pressure_sub, &dp);
    if (zero_dynamic_pressure_samples < NB_ZERO_SAMPLES_DYNAMIC_PRESSURE) {
        zero_dynamic_pressure_samples++;
        zero_dynamic_pressure_offset += dp.dynamic_pressure / NB_ZERO_SAMPLES_DYNAMIC_PRESSURE;
    } else {
        float airspeed = dynamic_pressure_to_airspeed(dp.dynamic_pressure - zero_dynamic_pressure_offset);
        float kinetic_energy = 0.5f*airspeed*airspeed; // per unit mass
        float dt = timestamp_duration(prev_kinetic_energy_timestamp, dp.timestamp_ns);
        float kinetic_energy_rate = (kinetic_energy - prev_kinetic_energy) / dt;
        float equivalent_climb_rate = kinetic_energy_rate / 9.81f;
        filtered_kinetic_energy_equivalent_climb_rate *= (1-KINETIC_ENERGY_FILTER_CONSTANT);
        filtered_kinetic_energy_equivalent_climb_rate += KINETIC_ENERGY_FILTER_CONSTANT * equivalent_climb_rate;
        log_debug("airspeed %f, kinetic_energy_rate %f, filtered %f", (double)airspeed, (double)kinetic_energy_rate, (double)filtered_kinetic_energy_equivalent_climb_rate);
        prev_kinetic_energy = kinetic_energy;
    }
    prev_kinetic_energy_timestamp = dp.timestamp_ns;
}

void vario_init(msgbus_t *bus)
{
    msgbus_topic_create(&_vario, bus, &vario_type, &_vario_buffer, "/vario");
}

void vario_register_tasks(msgbus_scheduler_t *sched)
{
    msgbus_t *bus = msgbus_scheduler_get_bus(sched);
    msgbus_topic_subscribe(&_static_pressure_sub, bus, "/sensors/ms5611", 1000000);
    msgbus_topic_subscribe(&_dynamic_pressure_sub, bus, "/sensors/ms4525do", 1000000);
    msgbus_subscriber_assert_type(&_static_pressure_sub, &barometer_sample_type);
    msgbus_subscriber_assert_type(&_dynamic_pressure_sub, &dynamic_pressure_sample_type);

    msgbus_scheduler_add_task(sched, &_dynamic_pressure_sub, dynamic_pressure_update, NULL);
    msgbus_scheduler_add_task(sched, &_static_pressure_sub, static_pressure_update, NULL);
}
