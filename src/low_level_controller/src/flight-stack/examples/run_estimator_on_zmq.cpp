#include <iostream>
#include "template_kalman.h"
#include "ekf_gyro_acc_mag.h"
#include <zmq.hpp>
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"


namespace ekf_mag {
    #include "code_gen/ekf_gyro_mag.h"
}

int main(int argc, char const *argv[])
{
    zmq::context_t context(1);

    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");

    subscriber.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);


    EKFGyroAccMag kalman;

    bool mag_calibration_ready = false;
    sleep(3);
    float prev_timestamp = 0;
    // int i = 1000;
    // int skip = 10;
    while (1) {
        zmq::message_t update;

        subscriber.recv(&update);


        // if (i > 0) {
        //     i--;
        //     continue;
        // }

        cmp_ctx_t cmp;
        cmp_mem_access_t mem;
        cmp_mem_access_ro_init(&cmp, &mem, update.data(), update.size());

        char strbuf[100];
        bool err = false;
        uint32_t strsize = sizeof(strbuf);
        err = err || !cmp_read_str(&cmp, &strbuf[0], &strsize);
        if (err) {
            continue;
        }
        // std::cerr << "received: " << strbuf << std::endl;
        if (strcmp("imu", strbuf) == 0) {
            uint32_t mapsize = 0;
            err = err | !cmp_read_map(&cmp, &mapsize);
            bool gyro_ok = false;
            bool acc_ok = false;
            bool time_ok = false;
            float gyro[3];
            float acc[3];
            float timestamp;
            for (int i = 0; i < mapsize; i++) {
                char strbuf[100];
                uint32_t strsize = sizeof(strbuf);
                err = err || !cmp_read_str(&cmp, strbuf, &strsize);
                uint32_t arr_sz;
                if (!err && strcmp("gyro", strbuf) == 0) {
                    err = err || !cmp_read_array(&cmp, &arr_sz);
                    err = err || (arr_sz != 3);
                    err = err || !cmp_read_float(&cmp, &gyro[0]);
                    err = err || !cmp_read_float(&cmp, &gyro[1]);
                    err = err || !cmp_read_float(&cmp, &gyro[2]);
                    gyro_ok = !err;
                } else if (!err && strcmp("acc", strbuf) == 0) {
                    err = err || !cmp_read_array(&cmp, &arr_sz);
                    err = err || (arr_sz != 3);
                    err = err || !cmp_read_float(&cmp, &acc[0]);
                    err = err || !cmp_read_float(&cmp, &acc[1]);
                    err = err || !cmp_read_float(&cmp, &acc[2]);
                    acc_ok = !err;
                } else if (!err && strcmp("time", strbuf) == 0) {
                    uint64_t int_timestamp;
                    err = err || !cmp_read_uinteger(&cmp, &int_timestamp);
                    if (!err) {
                        timestamp = ((float) int_timestamp) / 1000000;
                    }
                    time_ok = !err;
                }
            }

            // skip--;
            // if (skip > 0) {
            //     continue;
            // } else {
            //     skip = 10;
            // }

            if (!err && gyro_ok && acc_ok && time_ok) {
                acc[0] = acc[0]/1;
                acc[1] = acc[1]/1;
                acc[2] = acc[2]/1;

                if (prev_timestamp == 0 || !mag_calibration_ready) {
                    prev_timestamp = timestamp;
                    continue;
                }
                float delta_t = timestamp - prev_timestamp;
                prev_timestamp = timestamp;

                kalman.update_imu(gyro, acc, delta_t);

                std::cout << kalman.get_attitude().w() << ", "
                        << kalman.get_attitude().x() << ", "
                        << kalman.get_attitude().y() << ", "
                        << kalman.get_attitude().z() << std::endl;
                // std::cerr << "q: " << kalman.get_attitude().w() << ", "
                //         << kalman.get_attitude().x() << ", "
                //         << kalman.get_attitude().y() << ", "
                //         << kalman.get_attitude().z() << std::endl;

                // std::cerr << kalman.P << std::endl;
                // std::cerr << "time: " << timestamp << std::endl;

                // std::cerr << kalman.P(0, 0) << ", "
                //         << kalman.P(1, 1) << ", "
                //         << kalman.P(2, 2) << ", "
                //         << kalman.P(3, 3) << std::endl;

                // std::cerr << "imu: gyro: " << gyro[0] << ", " << gyro[1] << ", " << gyro[2];
                // std::cerr << "                                                 acc " << acc[0] << ", " << acc[1] << ", " << acc[2] << std::endl;
                // std::cerr << " time" << timestamp << std::endl;
            } else {
                std::cerr << "imu: msgpack read error : " << cmp_strerror(&cmp) << std::endl;
            }

        }
        if (strcmp("mag", strbuf) == 0) {
            static float mag_max[3] = {-1000, -1000, -1000};
            static float mag_min[3] = {1000, 1000, 1000};

            uint32_t mapsize = 0;
            err = err | !cmp_read_map(&cmp, &mapsize);
            bool field_ok = false;
            bool time_ok = false;
            float mag[3];
            uint64_t timestamp;
            for (int i = 0; i < mapsize; i++) {
                char strbuf[100];
                uint32_t strsize = sizeof(strbuf);
                err = err || !cmp_read_str(&cmp, strbuf, &strsize);
                uint32_t arr_sz;
                if (!err && strcmp("field", strbuf) == 0) {
                    err = err || !cmp_read_array(&cmp, &arr_sz);
                    err = err || (arr_sz != 3);
                    err = err || !cmp_read_float(&cmp, &mag[0]);
                    err = err || !cmp_read_float(&cmp, &mag[1]);
                    err = err || !cmp_read_float(&cmp, &mag[2]);
                    field_ok = !err;
                } else if (!err && strcmp("time", strbuf) == 0) {
                    err = err || !cmp_read_uinteger(&cmp, &timestamp);
                    time_ok = !err;
                }
            }

            if (!err && field_ok && time_ok) {
                // std::cerr << "mag field: " << mag[0] << ", " << mag[1] << ", " << mag[2] << std::endl;

                for (int i = 0; i < 3; i++) {
                    if (mag[i] > mag_max[i]) {
                        mag_max[i] = mag[i];
                    }
                    if (mag[i] < mag_min[i]) {
                        mag_min[i] = mag[i];
                    }
                }


                const float EARTH_MAG_FIELD_TH = 0.8*0.4;
                if (mag_max[0] - mag_min[0] > 2*EARTH_MAG_FIELD_TH
                    && mag_max[1] - mag_min[1] > EARTH_MAG_FIELD_TH
                    && mag_max[2] - mag_min[2] > EARTH_MAG_FIELD_TH) {

                    mag_calibration_ready = true;
                    std::cerr << "mag calibration ready" << std::endl;
                }

                float mag_zero[3];
                float mag_calib[3];
                for (int i = 0; i < 3; i++) {
                    mag_zero[i] = (mag_min[i] + mag_max[i]) / 2;
                    mag_calib[i] = mag[i] - mag_zero[i];
                }
                if (mag_calibration_ready) {

                    std::cerr << "calibrated mag field: " << mag_calib[0] << ", " << mag_calib[1] << ", " << mag_calib[2] << std::endl;
                    kalman.update_mag(&mag_calib[0]);

                    float mag_inertial[3];
                    ekf_mag::z_inertial(kalman.x[0], kalman.x[1], kalman.x[2], kalman.x[3], mag_calib[0], mag_calib[1], mag_calib[2], &mag_inertial[0]);
                    std::cerr << "meas " << mag_inertial[0] << ", " << mag_inertial[1] << ", " << mag_inertial[2] << std::endl;
                }
            }
        }
    }

    return 0;
}