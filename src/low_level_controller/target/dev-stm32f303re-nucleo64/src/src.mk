PROJINC += $(TARGETROOT)/src/
PROJINC += $(PROJROOT)/lib/comm/
PROJINC += $(PROJROOT)/lib/mcu/
PROJINC += $(PROJROOT)/lib/
PROJINC += $(PROJROOT)/lib/libnop/include
PROJINC += $(PROJROOT)/src
PROJINC += $(PROJROOT)/

PROJCPPSRC += $(TARGETROOT)/src/main.cpp
PROJCSRC += $(TARGETROOT)/src/usbcfg.c
PROJCSRC += $(TARGETROOT)/src/run_shell.c
PROJCSRC += $(TARGETROOT)/src/blocking_uart.c
PROJCSRC += $(TARGETROOT)/src/log.c
PROJCSRC += $(TARGETROOT)/src/timestamp.c
PROJCSRC += $(TARGETROOT)/src/timestamp_stm32.c

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
