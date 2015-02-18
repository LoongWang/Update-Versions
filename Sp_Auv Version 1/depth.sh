#!/bin/bash

if [ -z "$1" ]; then
	echo "usage: set_depth.sh <depth>"
    exit
fi

depth=$1

rostopic pub /DepthSounder std_msgs/Float32 $depth
