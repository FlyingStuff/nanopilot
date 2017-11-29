PROJINC += $(TARGETROOT)/src/
PROJINC += $(PROJROOT)/lib/comm/
PROJINC += $(PROJROOT)/lib/mcucom

PROJCSRC += $(TARGETROOT)/src/main.c
PROJCSRC += $(TARGETROOT)/src/usbcfg.c
PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram.c
PROJCSRC += $(PROJROOT)/lib/comm/serial-datagram/serial_datagram_buffer_writer.c
PROJCSRC += $(PROJROOT)/lib/comm/crc/crc32.c
PROJCSRC += $(PROJROOT)/lib/mcucom/net/net.c
