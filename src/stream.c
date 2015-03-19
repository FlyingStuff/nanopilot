#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "sensors/onboardsensors.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "serial-datagram/serial_datagram.h"

#include "stream.h"

#define EVENT_MASK_MPU6000 1

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
    static event_listener_t sensor_listener;
    chEvtRegisterMaskWithFlags(&sensor_events, &sensor_listener,
                               (eventmask_t)EVENT_MASK_MPU6000,
                               (eventflags_t)SENSOR_EVENT_MPU6000);
    static char dtgrm[100];
    static cmp_mem_access_t mem;
    static cmp_ctx_t cmp;
    while (1) {
        eventmask_t events = chEvtWaitAny(EVENT_MASK_MPU6000);
        float t = (float)chVTGetSystemTimeX() / CH_CFG_ST_FREQUENCY;
        if (events & EVENT_MASK_MPU6000) {
            chSysLock();
            float gx = mpu_gyro_sample.rate[0];
            float gy = mpu_gyro_sample.rate[1];
            float gz = mpu_gyro_sample.rate[2];
            float ax = mpu_acc_sample.acceleration[0];
            float ay = mpu_acc_sample.acceleration[1];
            float az = mpu_acc_sample.acceleration[2];
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
    }
    return 0;
}

void stream_start(BaseSequentialStream *out)
{
    chThdCreateStatic(stream_wa, sizeof(stream_wa), LOWPRIO, stream, out);
}