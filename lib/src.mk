
PROJINC += $(PROJROOT)/lib/math/eigen/
PROJINC += $(PROJROOT)/lib/
PROJINC += $(PROJROOT)/lib/comm
PROJINC += $(PROJROOT)/lib/mcu
PROJINC += $(PROJROOT)/lib/drivers
PROJINC += $(PROJROOT)/lib/mcucom
PROJINC += $(PROJROOT)/lib/mcucom/port/ChibiOS


PROJCSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/fault.c
PROJASMSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/fault_v7m.s
PROJCSRC += $(PROJROOT)/lib/mcu/arm-cortex-tools/mpu.c


PROJCSRC += $(PROJROOT)/lib/drivers/sensors/h3lis331dl.c
PROJCSRC += $(PROJROOT)/lib/drivers/sensors/hmc5883l.c
PROJCSRC += $(PROJROOT)/lib/drivers/sensors/mpu60X0.c
PROJCSRC += $(PROJROOT)/lib/drivers/sensors/ms5611.c
PROJCSRC += $(PROJROOT)/lib/comm/cmp/cmp.c
PROJCSRC += $(PROJROOT)/lib/comm/cmp_mem_access/cmp_mem_access.c
PROJCSRC += $(PROJROOT)/lib/comm/crc/crc32.c
PROJCSRC += $(PROJROOT)/lib/comm/datagram-messages/msg_dispatcher.c
PROJCSRC += $(PROJROOT)/lib/comm/datagram-messages/service_call.c
PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram.c
PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram_buffer_writer.c

PROJCSRC += $(PROJROOT)/lib/mcucom/msgbus/msgbus.c
PROJCSRC += $(PROJROOT)/lib/mcucom/port/ChibiOS/mcucom_port_sync.c
PROJCSRC += $(PROJROOT)/lib/mcucom/ts/serialization_msgpack.c
PROJCSRC += $(PROJROOT)/lib/mcucom/ts/type_print.c
