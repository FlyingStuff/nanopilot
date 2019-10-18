#!/bin/sh

# boot0
echo 203 > /sys/class/gpio/export
echo out >  /sys/class/gpio/gpio203/direction
echo 0 >  /sys/class/gpio/gpio203/value

# reset
echo 363 > /sys/class/gpio/export
echo out >  /sys/class/gpio/gpio363/direction
echo 1 >  /sys/class/gpio/gpio363/value
sleep 1
echo 0 >  /sys/class/gpio/gpio363/value
