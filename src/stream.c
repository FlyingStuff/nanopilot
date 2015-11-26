#include <ch.h>
#include <hal.h>
#include "thread_prio.h"
#include <chprintf.h>
#include <string.h>
#include "onboardsensors.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "serial-datagram/serial_datagram.h"
#include "attitude_determination.h"

#include "stream.h"

#define ONBOARDSENSOR_EVENT     1

static bool msg_header_write(cmp_ctx_t *cmp, const char *msg_id);
static void _stream_imu_values_sndfn(void *arg, const void *p, size_t len);


#define CMP_WRITE_C_STRING(cmp, str) cmp_write_str(&cmp, str, strlen(str))

static THD_WORKING_AREA(stream_wa, 512);
static THD_FUNCTION(stream, arg)
{
    BaseSequentialStream *out = (BaseSequentialStream*)arg;
    chRegSetThreadName("stream");
    static event_listener_t sensor_event_listener;
    chEvtRegisterMaskWithFlags(&sensor_events, &sensor_event_listener,
                               (eventmask_t)ONBOARDSENSOR_EVENT,
                               (eventflags_t)SENSOR_EVENT_MPU6000 | SENSOR_EVENT_H3LIS331DL | SENSOR_EVENT_HMC5883L | SENSOR_EVENT_MS5611);

    static char dtgrm[100];
    static cmp_mem_access_t mem;
    static cmp_ctx_t cmp;
    while (1) {
        eventmask_t events = chEvtWaitAny(ONBOARDSENSOR_EVENT);

        if (events & ONBOARDSENSOR_EVENT) {
            eventflags_t event_flags = chEvtGetAndClearFlags(&sensor_event_listener);
            if (event_flags & SENSOR_EVENT_MPU6000) {

                rate_gyro_sample_t gyro;
                accelerometer_sample_t acc;
                onboard_sensor_get_mpu6000_gyro_sample(&gyro);
                onboard_sensor_get_mpu6000_acc_sample(&acc);

                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "imu");
                err = err || !cmp_write_map(&cmp, 3);
                err = err || !CMP_WRITE_C_STRING(cmp, "gyro");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, gyro.rate[0]);
                err = err || !cmp_write_float(&cmp, gyro.rate[1]);
                err = err || !cmp_write_float(&cmp, gyro.rate[2]);
                err = err || !CMP_WRITE_C_STRING(cmp, "acc");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, acc.acceleration[0]);
                err = err || !cmp_write_float(&cmp, acc.acceleration[1]);
                err = err || !cmp_write_float(&cmp, acc.acceleration[2]);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_uint(&cmp, gyro.timestamp);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }


                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                err = false;
                float att[4];
                attitude_determination_get_attitude(att);
                err = err || !msg_header_write(&cmp, "att");
                err = err || !cmp_write_array(&cmp, 4);
                err = err || !cmp_write_float(&cmp, att[0]);
                err = err || !cmp_write_float(&cmp, att[1]);
                err = err || !cmp_write_float(&cmp, att[2]);
                err = err || !cmp_write_float(&cmp, att[3]);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }

            }
            if (event_flags & SENSOR_EVENT_HMC5883L) {

                magnetometer_sample_t magnetometer;
                onboard_sensor_get_hmc5883l_mag_sample(&magnetometer);

                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "mag");
                err = err || !cmp_write_map(&cmp, 2);
                err = err || !CMP_WRITE_C_STRING(cmp, "field");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, magnetometer.magnetic_field[0]);
                err = err || !cmp_write_float(&cmp, magnetometer.magnetic_field[1]);
                err = err || !cmp_write_float(&cmp, magnetometer.magnetic_field[2]);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_uint(&cmp, magnetometer.timestamp);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }
            }
            if (event_flags & SENSOR_EVENT_H3LIS331DL) {

                accelerometer_sample_t acc;
                onboard_sensor_get_h3lis331dl_acc_sample(&acc);

                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "hi_acc");
                err = err || !cmp_write_map(&cmp, 2);
                err = err || !CMP_WRITE_C_STRING(cmp, "acc");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, acc.acceleration[0]);
                err = err || !cmp_write_float(&cmp, acc.acceleration[1]);
                err = err || !cmp_write_float(&cmp, acc.acceleration[2]);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_uint(&cmp, acc.timestamp);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }
            }
            if (event_flags & SENSOR_EVENT_MS5611) {

                barometer_sample_t baro;
                onboard_sensor_get_ms5511_baro_sample(&baro);

                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "baro");
                err = err || !cmp_write_map(&cmp, 3);
                err = err || !CMP_WRITE_C_STRING(cmp, "static_press");
                err = err || !cmp_write_float(&cmp, baro.pressure);
                err = err || !CMP_WRITE_C_STRING(cmp, "air_temp");
                err = err || !cmp_write_float(&cmp, baro.temperature);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_uint(&cmp, baro.timestamp);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }
            }
        } // ONBOARDSENSOR_EVENT
    }
}


static void _stream_imu_values_sndfn(void *arg, const void *p, size_t len)
{
    if (len > 0) {
        chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)p, len);
    }
}


bool msg_header_write(cmp_ctx_t *cmp, const char *msg_id)
{
    bool err = false;
    err = err || !cmp_write_array(cmp, 2);
    err = err || !cmp_write_str(cmp, msg_id, strlen(msg_id));
    return !err;
}


void stream_start(BaseSequentialStream *out)
{
    if (out == NULL) {
        return;
    }
    chThdCreateStatic(stream_wa, sizeof(stream_wa), THD_PRIO_STREAM, stream, out);
}
