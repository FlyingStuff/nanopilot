#include <ch.h>
#include <string.h>
#include <memstreams.h>
#include <chprintf.h>
#include <ff.h>
#include "main.h"
#include "onboardsensors.h"
#include "sumd_input.h"
#include "git_revision.h"

#include "sdlog.h"

#define ONBOARDSENSOR_EVENT 1

static THD_WORKING_AREA(sdlog_wa, 512);
static THD_FUNCTION(sdlog, arg)
{
    (void)arg;
    chRegSetThreadName("sdlog");
    static event_listener_t sensor_listener;
    chEvtRegisterMaskWithFlags(&sensor_events, &sensor_listener,
                               (eventmask_t)ONBOARDSENSOR_EVENT,
                               (eventflags_t)SENSOR_EVENT_MPU6000);
    static uint8_t writebuf[200];
    static MemoryStream writebuf_stream;
    bool error = false;
    UINT _bytes_written;

    FRESULT res;

    static FIL version_fd;
    res = f_open(&version_fd, "/log/version", FA_WRITE | FA_CREATE_ALWAYS);
    if (res) {
        chprintf(stdout, "error %d opening %s\n", res, "/log/version");
        return -1;
    }
    msObjectInit(&writebuf_stream, writebuf, sizeof(writebuf), 0);
            chprintf((BaseSequentialStream*)&writebuf_stream,
                      "git version: %s (%s)\n"
                      "compiler:    %s\n"
                      "built:       %s\n",
                      build_git_version, build_git_branch,
                      PORT_COMPILER_NAME,
                      build_date);
    error = error || f_write(&version_fd, writebuf, writebuf_stream.eos, &_bytes_written);
    f_close(&version_fd);

    static FIL mpu6000_fd;
    res = f_open(&mpu6000_fd, "/log/mpu6000.csv", FA_WRITE | FA_CREATE_ALWAYS);
    if (res) {
        chprintf(stdout, "error %d opening %s\n", res, "/log/mpu6000.csv");
        return -1;
    }
    const char *mpu_descr = "time,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,temp\n";
    error |= error || f_write(&mpu6000_fd, mpu_descr, strlen(mpu_descr), &_bytes_written);
    static FIL rc_fd;
    res = f_open(&rc_fd, "/log/rc.csv", FA_WRITE | FA_CREATE_ALWAYS);
    if (res) {
        chprintf(stdout, "error %d opening %s\n", res, "/log/rc.csv");
        return -1;
    }
    const char *rc_descr = "time,signal,ch1,ch2,ch3,ch4,ch5\n";
    error |= error || f_write(&rc_fd, rc_descr, strlen(rc_descr), &_bytes_written);

    while (!error) {
        eventmask_t events = chEvtWaitAny(ONBOARDSENSOR_EVENT);
        float t = (float)chVTGetSystemTimeX() / CH_CFG_ST_FREQUENCY;

        if (events & ONBOARDSENSOR_EVENT) {
            chEvtGetAndClearFlags(&sensor_listener);
            chSysLock();
            float gx = onboard_mpu6000_gyro_sample.rate[0];
            float gy = onboard_mpu6000_gyro_sample.rate[1];
            float gz = onboard_mpu6000_gyro_sample.rate[2];
            float ax = onboard_mpu6000_acc_sample.acceleration[0];
            float ay = onboard_mpu6000_acc_sample.acceleration[1];
            float az = onboard_mpu6000_acc_sample.acceleration[2];
            float temp = onboard_mpu6000_temp;
            chSysUnlock();
            msObjectInit(&writebuf_stream, writebuf, sizeof(writebuf), 0);
            chprintf((BaseSequentialStream*)&writebuf_stream,
                      "%f,%f,%f,%f,%f,%f,%f,%f\n", t, gx, gy, gz, ax, ay, az, temp);
            UINT _bytes_written;
            int ret = f_write(&mpu6000_fd, writebuf, writebuf_stream.eos, &_bytes_written);
            if (ret != 0) {
                chprintf(stdout, "write failed %d\n", ret);
            }
            if (ret == 9) {
                f_open(&mpu6000_fd, "/log/mpu6000.csv", FA_WRITE);
                f_lseek(&mpu6000_fd, f_size(&mpu6000_fd));
            }
            static int sync_needed = 0;
            sync_needed++;
            if (sync_needed == 100) {
                f_sync(&mpu6000_fd);
                sync_needed = 0;
            }
        }
        if (false) {
            static struct rc_input_s rc_in;
            sumd_input_get(&rc_in);
            msObjectInit(&writebuf_stream, writebuf, sizeof(writebuf), 0);
            chprintf((BaseSequentialStream*)&writebuf_stream,
                      "%f,%d,%f,%f,%f,%f,%f\n", t, !rc_in.no_signal, rc_in.channel[0], rc_in.channel[1], rc_in.channel[2], rc_in.channel[3], rc_in.channel[4]);
            UINT _bytes_written;
            int ret = f_write(&rc_fd, writebuf, writebuf_stream.eos, &_bytes_written);
            if (ret != 0) {
                chprintf(stdout, "write failed %d\n", ret);
            }
            static int sync_needed = 0;
            sync_needed++;
            if (sync_needed == 100) {
                f_sync(&rc_fd);
                sync_needed = 0;
            }
        }
    }
    return -1;
}

void sdlog_start(void)
{
    chThdCreateStatic(sdlog_wa, sizeof(sdlog_wa), LOWPRIO, sdlog, NULL);
}