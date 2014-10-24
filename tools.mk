.PHONY: flash
flash: all
	openocd -f oocd.cfg -c "program build/ins-board.elf verify reset"

.PHONY: r
r: reset
.PHONY: reset
reset:
	openocd -f oocd.cfg -c "init" -c "reset" -c "shutdown"
