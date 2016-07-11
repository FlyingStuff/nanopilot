#!/bin/sh

cd src/attitude_estimation/code_gen/
python3 ekf_gyro_acc.py -l info
python3 ekf_gyro_acc_mag.py -l info
