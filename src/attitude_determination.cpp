#include <ch.h>
#include "thread_prio.h"
#include "onboardsensors.h"
#include "main.h"
#include "chprintf.h"

#include "attitude_estimation/ekf_gyro_acc_mag.h"

static Eigen::Quaternionf attitude;



typedef struct {
    float max[3];
    float min[3];
} magneto_calib_t;

static void magneto_calibration_init(magneto_calib_t *calib)
{
    for (int i = 0; i < 3; i++) {
        calib->max[i] = -10000;
        calib->min[i] = 10000;
    }
}

static bool magneto_calibration_update(magneto_calib_t *calib, float *mag)
{
    for (int i = 0; i < 3; i++) {
        if (mag[i] > calib->max[i]) {
            calib->max[i] = mag[i];
        }
        if (mag[i] < calib->min[i]) {
            calib->min[i] = mag[i];
        }
    }

    const float EARTH_MAG_FIELD_TH = 0.8*0.4;
    if (calib->max[0] - calib->min[0] > 2*EARTH_MAG_FIELD_TH
        && calib->max[1] - calib->min[1] > EARTH_MAG_FIELD_TH
        && calib->max[2] - calib->min[2] > EARTH_MAG_FIELD_TH) {

        return true;
    }
    return false;
}

static void magneto_calibration_apply(magneto_calib_t *calib, float *mag)
{
    for (int i = 0; i < 3; i++) {
        mag[i] = mag[i] - (calib->min[i] + calib->max[i]) / 2;
    }
}


#define ONBOARDSENSOR_EVENT     1

static THD_WORKING_AREA(attitude_determination_wa, 1024);
static THD_FUNCTION(attitude_determination, arg)
{
    (void)arg;
    chRegSetThreadName("attitude determination");
    chThdSleepMilliseconds(100);


    event_listener_t sensor_event_listener;
    chEvtRegisterMaskWithFlags(&sensor_events, &sensor_event_listener,
                               (eventmask_t)ONBOARDSENSOR_EVENT,
                               (eventflags_t)SENSOR_EVENT_MPU6000 | SENSOR_EVENT_HMC5883L);


    EKFGyroAccMag attitude_estimator;

    magneto_calib_t mag_calib;
    magneto_calibration_init(&mag_calib);
    timestamp_t prev_mpu6000_timestamp = 0;
    while (1) {
        eventmask_t events = chEvtWaitAny(ONBOARDSENSOR_EVENT);
        if (events & ONBOARDSENSOR_EVENT) {
            eventflags_t event_flags = chEvtGetAndClearFlags(&sensor_event_listener);
            if (event_flags & SENSOR_EVENT_MPU6000) {
                rate_gyro_sample_t gyro;
                accelerometer_sample_t acc;
                onboard_sensor_get_mpu6000_gyro_sample(&gyro);
                onboard_sensor_get_mpu6000_acc_sample(&acc);

                if (prev_mpu6000_timestamp == 0) {
                    prev_mpu6000_timestamp = gyro.timestamp;
                } else {
                    float delta_t = timestamp_duration_s(prev_mpu6000_timestamp, gyro.timestamp);
                    prev_mpu6000_timestamp = gyro.timestamp;

                    attitude_estimator.update_imu(gyro.rate, acc.acceleration, delta_t);
                    chSysLock();
                    attitude = attitude_estimator.get_attitude();
                    chSysUnlock();
                }
            }
            if (event_flags & SENSOR_EVENT_HMC5883L) {
                magnetometer_sample_t mag;
                onboard_sensor_get_hmc5883l_mag_sample(&mag);
                if (magneto_calibration_update(&mag_calib, &mag.magnetic_field[0])) {
                    status_led_toggle();
                    magneto_calibration_apply(&mag_calib, &mag.magnetic_field[0]);
                    // chprintf(stdout, "calibrated field %f %f %f\n",mag.magnetic_field[0], mag.magnetic_field[1], mag.magnetic_field[2]);
                    attitude_estimator.update_mag(&mag.magnetic_field[0]);
                }
            }
        }
    }
}

extern "C" void run_attitude_determination(void)
{
    chThdCreateStatic(attitude_determination_wa, sizeof(attitude_determination_wa), THD_PRIO_SENSOR_ATTITUDE_DETERMINATION, attitude_determination, NULL);
}


extern "C" void attitude_determination_get_attitude(float *quaternion)
{
    chSysLock();
    quaternion[0] = attitude.w();
    quaternion[1] = attitude.x();
    quaternion[2] = attitude.y();
    quaternion[3] = attitude.z();
    chSysUnlock();
}
