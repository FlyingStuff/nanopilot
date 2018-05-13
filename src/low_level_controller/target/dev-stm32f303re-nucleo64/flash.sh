#!/bin/sh

st-flash --reset write build/stm32f303re-nucleo64.bin 0x8000000
