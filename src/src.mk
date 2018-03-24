
PROJINC += $(PROJROOT)/src/
PROJINC += $(PROJROOT)/src/flight-stack/src/



PROJCPPSRC += $(PROJROOT)/src/flight-stack/src/attitude_estimation/ekf_gyro_acc.cpp
PROJCPPSRC += $(PROJROOT)/src/flight-stack/src/attitude_estimation/ekf_gyro_acc_mag.cpp

PROJCSRC += $(PROJROOT)/src/error.c
PROJCSRC += $(PROJROOT)/src/blocking_uart.c
PROJCPPSRC += $(PROJROOT)/src/attitude_determination.cpp
PROJCSRC += $(PROJROOT)/src/datagram_message_comm.c
PROJCSRC += $(PROJROOT)/src/log.c
PROJCSRC += $(PROJROOT)/src/onboardsensors.c
PROJCSRC += $(PROJROOT)/src/ms4525do_publisher.c
PROJCSRC += $(PROJROOT)/src/sdcard.c
PROJCSRC += $(PROJROOT)/src/sdlog.c
PROJCSRC += $(PROJROOT)/src/sqrtpatch.c
PROJCSRC += $(PROJROOT)/src/stream.c
PROJCSRC += $(PROJROOT)/src/sumd_input.c
PROJCSRC += $(PROJROOT)/src/syscalls.c
PROJCSRC += $(PROJROOT)/src/timestamp_stm32.c
PROJCSRC += $(PROJROOT)/src/hott/sumd.c
PROJCSRC += $(PROJROOT)/src/hott/telemetry.c
PROJCSRC += $(PROJROOT)/src/hott_tm.c
PROJCSRC += $(PROJROOT)/src/msgbus_scheduler.c
PROJCSRC += $(PROJROOT)/src/rate_limiter.c
PROJCSRC += $(PROJROOT)/src/timestamp.c
PROJCSRC += $(PROJROOT)/src/airdata.c
PROJCSRC += $(PROJROOT)/src/analog.c
