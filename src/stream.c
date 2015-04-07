#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include "onboardsensors.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "serial-datagram/serial_datagram.h"

#include "stream.h"

#define ONBOARDSENSOR_EVENT     1

static void _stream_imu_values_sndfn(void *arg, const void *p, size_t len)
{
    if (len > 0) {
        chSequentialStreamWrite((BaseSequentialStream*)arg, (const uint8_t*)p, len);
    }
}

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
                err = err || !cmp_write_map(&cmp, 3);
                const char *gyro_id = "gyro";
                err = err || !cmp_write_str(&cmp, gyro_id, strlen(gyro_id));
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, gx);
                err = err || !cmp_write_float(&cmp, gy);
                err = err || !cmp_write_float(&cmp, gz);
                const char *acc_id = "acc";
                err = err || !cmp_write_str(&cmp, acc_id, strlen(acc_id));
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, ax);
                err = err || !cmp_write_float(&cmp, ay);
                err = err || !cmp_write_float(&cmp, az);
                const char *time_id = "time";
                err = err || !cmp_write_str(&cmp, time_id, strlen(time_id));
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
                err = err || !cmp_write_map(&cmp, 2);
                const char *mag_id = "magfield";
                err = err || !cmp_write_str(&cmp, mag_id, strlen(mag_id));
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, mx);
                err = err || !cmp_write_float(&cmp, my);
                err = err || !cmp_write_float(&cmp, mz);
                const char *time_id = "time";
                err = err || !cmp_write_str(&cmp, time_id, strlen(time_id));
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
                err = err || !cmp_write_map(&cmp, 2);
                const char *acc_id = "hi_acc";
                err = err || !cmp_write_str(&cmp, acc_id, strlen(acc_id));
                err = err || !cmp_write_array(&cmp, 3);
                err = err || !cmp_write_float(&cmp, ax);
                err = err || !cmp_write_float(&cmp, ay);
                err = err || !cmp_write_float(&cmp, az);
                const char *time_id = "time";
                err = err || !cmp_write_str(&cmp, time_id, strlen(time_id));
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
                err = err || !cmp_write_map(&cmp, 3);
                const char *press_id = "static_press";
                err = err || !cmp_write_str(&cmp, press_id, strlen(press_id));
                err = err || !cmp_write_float(&cmp, baro);
                const char *temp_id = "air_temp";
                err = err || !cmp_write_str(&cmp, temp_id, strlen(temp_id));
                err = err || !cmp_write_float(&cmp, temp);
                const char *time_id = "time";
                err = err || !cmp_write_str(&cmp, time_id, strlen(time_id));
                err = err || !cmp_write_float(&cmp, t);
                if (!err) {
                    serial_datagram_send(dtgrm, cmp_mem_access_get_pos(&mem), _stream_imu_values_sndfn, out);
                }
            }
        } // ONBOARDSENSOR_EVENT
    }
    return 0;
}

void stream_start(BaseSequentialStream *out)
{
    chThdCreateStatic(stream_wa, sizeof(stream_wa), LOWPRIO, stream, out);
}