#!/bin/sh

st-flash --reset write build/stm32f429zi-nucleo144.bin 0x8000000
