#!/bin/sh

export LD_LIBRARY_PATH=/home/pi/opencv/opencv/build/lib
v4l2-ctl --set-ctrl=exposure_auto=1

echo "There" > /home/pi/Log

/home/pi/CHS2017/ColorLocator/ColorLocator &

