
PROJINC += $(PROJROOT)/src/eigen/
PROJINC += $(PROJROOT)/src/
PROJINC += $(PROJROOT)/src/flight-stack/src/
PROJINC += $(PROJROOT)/src/msgbus/ports/ChibiOS


PROJCSRC += $(PROJROOT)/src/arm-cortex-tools/fault.c
PROJASMSRC += $(PROJROOT)/src/arm-cortex-tools/fault_v7m.s
PROJCSRC += $(PROJROOT)/src/arm-cortex-tools/mpu.c


PROJCPPSRC += $(PROJROOT)/src/flight-stack/src/attitude_estimation/ekf_gyro_acc.cpp
PROJCPPSRC += $(PROJROOT)/src/flight-stack/src/attitude_estimation/ekf_gyro_acc_mag.cpp

PROJCSRC += $(PROJROOT)/src/error.c
PROJCPPSRC += $(PROJROOT)/src/attitude_determination.cpp
PROJCSRC += $(PROJROOT)/src/datagram_message_comm.c
PROJCSRC += $(PROJROOT)/src/log.c
PROJCSRC += $(PROJROOT)/src/onboardsensors.c
PROJCSRC += $(PROJROOT)/src/sdcard.c
PROJCSRC += $(PROJROOT)/src/sdlog.c
PROJCSRC += $(PROJROOT)/src/sensors/h3lis331dl.c
PROJCSRC += $(PROJROOT)/src/sensors/hmc5883l.c
PROJCSRC += $(PROJROOT)/src/sensors/mpu60X0.c
PROJCSRC += $(PROJROOT)/src/sensors/ms5611.c
PROJCSRC += $(PROJROOT)/src/shell_cmds.c
PROJCSRC += $(PROJROOT)/src/sqrtpatch.c
PROJCSRC += $(PROJROOT)/src/stream.c
PROJCSRC += $(PROJROOT)/src/sumd_input.c
PROJCSRC += $(PROJROOT)/src/syscalls.c
PROJCSRC += $(PROJROOT)/src/timestamp/timestamp_stm32.c
PROJCSRC += $(PROJROOT)/src/usbcfg.c
PROJCSRC += $(PROJROOT)/src/cmp/cmp.c
PROJCSRC += $(PROJROOT)/src/cmp_mem_access/cmp_mem_access.c
PROJCSRC += $(PROJROOT)/src/crc/crc32.c
PROJCSRC += $(PROJROOT)/src/datagram-messages/msg_dispatcher.c
PROJCSRC += $(PROJROOT)/src/datagram-messages/service_call.c
PROJCSRC += $(PROJROOT)/src/hott/sumd.c
PROJCSRC += $(PROJROOT)/src/msgbus/msgbus.c
PROJCSRC += $(PROJROOT)/src/msgbus/ports/ChibiOS/msgbus_port.c
PROJCSRC += $(PROJROOT)/src/msgbus/serialization_msgpack.c
PROJCSRC += $(PROJROOT)/src/msgbus/type_print.c
PROJCSRC += $(PROJROOT)/src/msgbus_scheduler.c
PROJCSRC += $(PROJROOT)/src/parameter/parameter.c
PROJCSRC += $(PROJROOT)/src/parameter/parameter_msgpack.c
PROJCSRC += $(PROJROOT)/src/parameter/parameter_print.c
PROJCSRC += $(PROJROOT)/src/rate_limiter.c
PROJCSRC += $(PROJROOT)/src/rcservo/rcservo.c
PROJCSRC += $(PROJROOT)/src/serial-datagram/serial_datagram.c
PROJCSRC += $(PROJROOT)/src/serial-datagram/serial_datagram_buffer_writer.c
PROJCSRC += $(PROJROOT)/src/timestamp/timestamp.c
