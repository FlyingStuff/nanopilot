
flash:
	openocd -f oocd.cfg -c "program build/ins-board.elf verify reset"
