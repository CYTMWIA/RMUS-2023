#!/bin/bash

xhost +

docker network create net-sim

docker run -it --rm --name ros-noetic --network net-sim \
	-v $(pwd)/:/ws \
	-e ROS_MASTER_URI=http://ros-master:11311 \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e NO_AT_BRIDGE=1 \
	-e LIBGL_ALWAYS_SOFTWARE=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
    osrf/ros:noetic-desktop-full-focal bash
