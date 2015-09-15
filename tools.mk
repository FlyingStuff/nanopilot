.PHONY: flash
flash: all
	openocd -f oocd.cfg -c "program build/ins-board.elf verify reset" -c "shutdown"

.PHONY: r
r: reset
.PHONY: reset
reset:
	openocd -f oocd.cfg -c "init" -c "reset" -c "shutdown"

.PHONY: dfu
dfu: all
	dfu-util -d 0483:df11 -c 1 -i 0 -a 0 --dfuse-address 0x08000000 -R -D build/ins-board.bin
