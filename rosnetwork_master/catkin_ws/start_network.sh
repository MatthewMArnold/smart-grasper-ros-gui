#!/usr/bin/env sh
#
# Starts the network
# Note: must call roscore before calling this script

rosrun serial_pkg serial_node.py &  # starts the serial handler
rosrun gui_pkg gui_pkg &            # starts gui
