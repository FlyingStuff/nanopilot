#include <ch.h>
#include "thread_prio.h"
#include "main.h"
#include "attitude_estimation/ekf_gyro_acc_mag.h"
#include "msgbus/typesafe_wrapper.h"
#include "types/sensors.h"
#include "types/geometry.h"
#include "timestamp/timestamp.h"
#include "log.h"
#include <cmath>

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

    const float EARTH_MAG_FIELD_TH = 0.8 * 0.4 * 0.0001;
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



static THD_WORKING_AREA(attitude_determination_wa, 2048);
static THD_FUNCTION(attitude_determination, arg)
{
    (void)arg;
    chRegSetThreadName("attitude determination");

    static EKFGyroAccMag attitude_estimator;
    static MsgBusTopic(pose) att_pub;
    att_pub.advertise(&bus, "/attitude_estimator_inertial/attitude_body_to_world");

    static MsgBusSubscriber(mems_imu_sample) imu_sub;
    static MsgBusSubscriber(magnetometer_sample) mag_sub;
    imu_sub.subscribe(&bus, "/sensors/mpu6000");
    mag_sub.subscribe(&bus, "/sensors/hmc5883l");

    static magneto_calib_t mag_calib;
    magneto_calibration_init(&mag_calib);
    uint64_t prev_mpu6000_timestamp = 0;

    while (1) {
        msgbus_subscriber_t *inputs[] = {&imu_sub.sub, &mag_sub.sub};
        msgbus_subscriber_wait_for_update_on_any(inputs, 2, MSGBUS_TIMEOUT_NEVER);
        if (imu_sub.has_update()) {
            mems_imu_sample_t imu;
            imu_sub.read(imu);
            if (prev_mpu6000_timestamp == 0) {
                prev_mpu6000_timestamp = imu.timestamp;
            } else {
                float delta_t = ltimestamp_duration_s(prev_mpu6000_timestamp, imu.timestamp);
                prev_mpu6000_timestamp = imu.timestamp;
                attitude_estimator.update_imu(imu.rate, imu.acceleration, delta_t);
                Eigen::Quaternionf attitude = attitude_estimator.get_attitude();
                pose_t pose;
                pose.attitude.r = attitude.w();
                pose.attitude.i = attitude.x();
                pose.attitude.j = attitude.y();
                pose.attitude.k = attitude.z();
                pose.position.x = NAN;
                pose.position.y = NAN;
                pose.position.z = NAN;
                pose.timestamp = imu.timestamp;
                att_pub.publish(pose);
            }
        }
        if (mag_sub.has_update()) {
            magnetometer_sample_t mag;
            mag_sub.read(mag);
            if (magneto_calibration_update(&mag_calib, &mag.magnetic_field[0])) {
                status_led_toggle();
                magneto_calibration_apply(&mag_calib, &mag.magnetic_field[0]);
                attitude_estimator.update_mag(&mag.magnetic_field[0]);
            }
        }
    }

}

extern "C" void run_attitude_determination(void)
{
    chThdCreateStatic(attitude_determination_wa, sizeof(attitude_determination_wa), THD_PRIO_SENSOR_ATTITUDE_DETERMINATION, attitude_determination, NULL);
}

