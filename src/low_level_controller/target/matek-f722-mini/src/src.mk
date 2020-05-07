PROJINC += $(TARGETROOT)/src/
PROJINC += $(PROJROOT)/lib/comm/
PROJINC += $(PROJROOT)/lib/mcu/
PROJINC += $(PROJROOT)/lib/
PROJINC += $(PROJROOT)/lib/eigen/
PROJINC += $(PROJROOT)/lib/libnop/include
PROJINC += $(PROJROOT)/src
PROJINC += $(PROJROOT)/

PROJCSRC += $(TARGETROOT)/src/board.c
PROJCSRC += $(TARGETROOT)/src/usbcfg.c
PROJCPPSRC += $(TARGETROOT)/src/main.cpp
PROJCPPSRC += $(TARGETROOT)/src/actuators.cpp
PROJCPPSRC += $(TARGETROOT)/src/adc.cpp
PROJCSRC += $(TARGETROOT)/src/run_shell.c


PROJCSRC += $(PROJROOT)/src/blocking_uart.c
PROJCSRC += $(PROJROOT)/src/parameter_port.c
PROJCSRC += $(PROJROOT)/src/log.c
PROJCSRC += $(PROJROOT)/src/timestamp.c
PROJCSRC += $(PROJROOT)/src/timestamp_stm32.c
PROJCSRC += $(PROJROOT)/src/hott/sumd.c
PROJCSRC += $(PROJROOT)/src/hott/telemetry.c
PROJCSRC += $(PROJROOT)/src/pid.c
PROJCSRC += $(PROJROOT)/src/drivers/lis3mdl.c
PROJCSRC += $(PROJROOT)/src/drivers/icm20602.c
PROJCPPSRC += $(PROJROOT)/src/parameter_storage.cpp
PROJCPPSRC += $(PROJROOT)/src/eeprom.cpp
PROJCPPSRC += $(PROJROOT)/src/hott_tm.cpp
PROJCPPSRC += $(PROJROOT)/src/rc_input.cpp
PROJCPPSRC += $(PROJROOT)/src/sumd_input.cpp
PROJCPPSRC += $(PROJROOT)/src/rc_pwm_out.cpp
PROJCPPSRC += $(PROJROOT)/src/ros_comm.cpp
PROJCPPSRC += $(PROJROOT)/src/pid_with_parameter.cpp
PROJCPPSRC += $(PROJROOT)/src/low_pass_filter.cpp
PROJCPPSRC += $(PROJROOT)/src/control_loop.cpp
PROJCPPSRC += $(PROJROOT)/src/rate_controller_pid.cpp
PROJCPPSRC += $(PROJROOT)/src/output_mixer_linear.cpp
PROJCPPSRC += $(PROJROOT)/src/lis3mdl_publisher.cpp
PROJCPPSRC += $(PROJROOT)/src/icm20602_publisher.cpp
PROJCPPSRC += $(PROJROOT)/src/arm_led.cpp
PROJCPPSRC += $(PROJROOT)/src/sensors.cpp


PROJCSRC += $(PROJROOT)/src/syscalls.c
PROJCPPSRC += $(PROJROOT)/src/shell_cmds.cpp
PROJCSRC += $(PROJROOT)/src/panic_handler.c

PROJCSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/fault.c
PROJCSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/mpu.c
PROJASMSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/fault_v7m.s

PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram.c
PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram_buffer_writer.c
PROJCSRC += $(PROJROOT)/lib/comm/crc/crc32.c

PROJCSRC += $(PROJROOT)/lib/parameter/parameter.c
PROJCSRC += $(PROJROOT)/lib/parameter/parameter_msgpack.c
PROJCSRC += $(PROJROOT)/lib/parameter/parameter_print.c

PROJCSRC += $(PROJROOT)/lib/comm/cmp/cmp.c
PROJCSRC += $(PROJROOT)/lib/comm/cmp_mem_access/cmp_mem_access.c

PROJCPPSRC += $(PROJROOT)/ros_interface/comm.cpp
