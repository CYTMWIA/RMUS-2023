#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from functools import partial
from math import pi as PI
from typing import List

import rospy
from geometry_msgs.msg import Pose, Twist
from navi import EpPose, Navi
from rmus_solution.srv import graspsignal, setgoal, switch
from std_msgs.msg import Int32MultiArray, UInt8MultiArray
from rmus_solution.msg import SquareArray
# 预定义的路径点、名称: 位置(pose_x, pose_y, yaw), 误差容限(pose, angle)
PRE_DEFINED_POSE = {
    "home": EpPose(0.00, 0.00, 0.00, 0.02, 0.05),
    "noticeboard": EpPose(0.00, 1.60, 0.00),
    "station-1": EpPose(1.15, 1.91, 0.00, 0.075, 0.1),
    "station-2": EpPose(1.15, 1.80, 0.00, 0.075, 0.1),
    "station-3": EpPose(1.15, 1.65, 0.00, 0.075, 0.1),
}


def wait_for_services(services: List[str]):
    for ser in services:
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(ser, 1.0)
                break
            except:
                rospy.logwarn(f"Waiting for '{ser}' Service")
                rospy.sleep(0.5)


def make_speed_setter(cmd_vel: str):
    pub = rospy.Publisher(cmd_vel, Twist, queue_size=1)
    def func(x, y, az):
        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = az
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        pub.publish(twist)
    return func

def get_target_blocks():
    while not rospy.is_shutdown():
        sqrs: SquareArray = rospy.wait_for_message("/squares", SquareArray)
        high_sqrs= []
        for sqr in sqrs.data:
            h = sum([p.z for p in sqr.points])/4
            if h>0.4:
                ya = sum([p.y for p in sqr.points])/4
                high_sqrs.append((sqr.id, ya))
        if len(high_sqrs)==3:
            break
    high_sqrs.sort(key=lambda s: s[1], reverse=True)
    return [q[0] for q in high_sqrs]
        
if __name__ == '__main__':
    rospy.init_node("gamecore_node")

    set_speed = make_speed_setter("/cmd_vel")
    navi = Navi()

    navi.goto(PRE_DEFINED_POSE["noticeboard"])
    targets = get_target_blocks()
    rospy.loginfo(f"TARGETS: {targets}")

