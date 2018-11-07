PROJINC += $(TARGETROOT)/src/
PROJINC += $(PROJROOT)/lib/comm/
PROJINC += $(PROJROOT)/lib/mcu/
PROJINC += $(PROJROOT)/lib/
PROJINC += $(PROJROOT)/lib/libnop/include
PROJINC += $(PROJROOT)/src
PROJINC += $(PROJROOT)/

PROJCPPSRC += $(TARGETROOT)/src/main.cpp
PROJCPPSRC += $(TARGETROOT)/src/actuators.cpp
PROJCSRC += $(TARGETROOT)/src/usbcfg.c
PROJCSRC += $(TARGETROOT)/src/run_shell.c


PROJCSRC += $(PROJROOT)/src/blocking_uart.c
PROJCSRC += $(PROJROOT)/src/log.c
PROJCSRC += $(PROJROOT)/src/timestamp.c
PROJCSRC += $(PROJROOT)/src/timestamp_stm32.c
PROJCSRC += $(PROJROOT)/src/hott/sumd.c
PROJCSRC += $(PROJROOT)/src/hott/telemetry.c
PROJCPPSRC += $(PROJROOT)/src/sumd_input.cpp
PROJCPPSRC += $(PROJROOT)/src/rc_pwm_out.cpp

PROJCSRC += $(PROJROOT)/src/syscalls.c
PROJCSRC += $(PROJROOT)/src/shell_cmds.c
PROJCSRC += $(PROJROOT)/src/panic_handler.c

PROJCSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/fault.c
PROJCSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/mpu.c
PROJASMSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/fault_v7m.s

PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram.c
PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram_buffer_writer.c
PROJCSRC += $(PROJROOT)/lib/comm/crc/crc32.c

PROJCSRC += $(PROJROOT)/lib/parameter/parameter.c
PROJCSRC += $(PROJROOT)/lib/parameter/parameter_print.c

PROJCPPSRC += $(PROJROOT)/ros_interface/comm.cpp
