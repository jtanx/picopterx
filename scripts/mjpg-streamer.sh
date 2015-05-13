#!/bin/bash

export LD_LIBRARY_PATH=/usr/local/lib

pkill mjpg_streamer
v4l2-ctl --set-fmt-video=width=320,height=240
v4l2-ctl --set-parm 30
mjpg_streamer -i 'input_file.so -f /mnt/ramdisk' &
echo ' '
