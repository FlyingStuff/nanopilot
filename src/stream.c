#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include "onboardsensors.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "serial-datagram/serial_datagram.h"

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
        float t = (float)chVTGetSystemTimeX() / CH_CFG_ST_FREQUENCY;
        if (events & ONBOARDSENSOR_EVENT) {
            eventflags_t event_flags = chEvtGetAndClearFlags(&sensor_event_listener);
            if (event_flags & SENSOR_EVENT_MPU6000) {
                chSysLock();
                float gx = onboard_mpu6000_gyro_sample.rate[0];
                float gy = onboard_mpu6000_gyro_sample.rate[1];
                float gz = onboard_mpu6000_gyro_sample.rate[2];
                float ax = onboard_mpu6000_acc_sample.acceleration[0];
                float ay = onboard_mpu6000_acc_sample.acceleration[1];
                float az = onboard_mpu6000_acc_sample.acceleration[2];
                chSysUnlock();
                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "imu");
                err = err || !cmp_write_map(&cmp, 3);
                err = err || !CMP_WRITE_C_STRING(cmp, "gyro");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, gx);
                err = err || !cmp_write_float(&cmp, gy);
                err = err || !cmp_write_float(&cmp, gz);
                err = err || !CMP_WRITE_C_STRING(cmp, "acc");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, ax);
                err = err || !cmp_write_float(&cmp, ay);
                err = err || !cmp_write_float(&cmp, az);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_float(&cmp, t);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }
            }
            if (event_flags & SENSOR_EVENT_HMC5883L) {
                chSysLock();
                float mx = onboard_hmc5883l_mag_sample.magnetic_field[0];
                float my = onboard_hmc5883l_mag_sample.magnetic_field[1];
                float mz = onboard_hmc5883l_mag_sample.magnetic_field[2];
                chSysUnlock();
                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "mag");
                err = err || !cmp_write_map(&cmp, 2);
                err = err || !CMP_WRITE_C_STRING(cmp, "field");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, mx);
                err = err || !cmp_write_float(&cmp, my);
                err = err || !cmp_write_float(&cmp, mz);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_float(&cmp, t);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }
            }
            if (event_flags & SENSOR_EVENT_H3LIS331DL) {
                chSysLock();
                float ax = onboard_h3lis331dl_acc_sample.acceleration[0];
                float ay = onboard_h3lis331dl_acc_sample.acceleration[1];
                float az = onboard_h3lis331dl_acc_sample.acceleration[2];
                chSysUnlock();
                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "hi_acc");
                err = err || !cmp_write_map(&cmp, 2);
                err = err || !CMP_WRITE_C_STRING(cmp, "acc");
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, ax);
                err = err || !cmp_write_float(&cmp, ay);
                err = err || !cmp_write_float(&cmp, az);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_float(&cmp, t);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }
            }
            if (event_flags & SENSOR_EVENT_MS5611) {
                chSysLock();
                float baro = onboard_ms5511_baro_sample.pressure;
                float temp = onboard_ms5511_baro_sample.temperature;
                chSysUnlock();
                cmp_mem_access_init(&cmp, &mem, dtgrm, sizeof(dtgrm));
                bool err = false;
                err = err || !msg_header_write(&cmp, "baro");
                err = err || !cmp_write_map(&cmp, 3);
                err = err || !CMP_WRITE_C_STRING(cmp, "static_press");
                err = err || !cmp_write_float(&cmp, baro);
                err = err || !CMP_WRITE_C_STRING(cmp, "air_temp");
                err = err || !cmp_write_float(&cmp, temp);
                err = err || !CMP_WRITE_C_STRING(cmp, "time");
                err = err || !cmp_write_float(&cmp, t);
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
    chThdCreateStatic(stream_wa, sizeof(stream_wa), LOWPRIO, stream, out);
}