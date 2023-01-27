#!/bin/bash
docker exec -it client /opt/ros/noetic/env.sh /opt/workspace/devel_isolated/env.sh /opt/ep_ws/devel/env.sh rostopic pub -1 /pose_set geometry_msgs/PoseArray "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
poses:
- position:
    x: 4.0
    y: 0.0
    z: 2.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: 3.0
    y: 0.0
    z: 3.15
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: 3.0
    y: 0.0
    z: 3.3
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: 3.0
    y: 0.0
    z: 3.45
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
- position:
    x: 3.0
    y: 0.0
    z: 3.6
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0

"