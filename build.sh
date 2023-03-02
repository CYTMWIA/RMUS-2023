#! /bin/bash

# This script should be run in docker image building

# copy from original Dockerfile
source /opt/workspace/devel_isolated/setup.bash
catkin_make install --use-ninja -DSETUPTOOLS_DEB_LAYOUT=OFF
