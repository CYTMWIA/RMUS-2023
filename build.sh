#!/bin/bash

if [ -f /.dockerenv ]; then
    # copy from original Dockerfile
    source /opt/workspace/devel_isolated/setup.bash
else
    source /opt/ros/noetic/setup.sh
fi

catkin_make install --use-ninja -DSETUPTOOLS_DEB_LAYOUT=OFF
