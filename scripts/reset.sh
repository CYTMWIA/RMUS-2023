#!/bin/bash

docker exec -it client /opt/ros/noetic/env.sh /opt/workspace/devel_isolated/env.sh /opt/ep_ws/devel/env.sh rostopic pub -1 /reset geometry_msgs/Point "{x: 0.0, y: 0.0, z: 0.0}"