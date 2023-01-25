#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from functools import partial
from math import pi as PI
from typing import List

import rospy
from geometry_msgs.msg import Twist
from navi import EpPose, Navi
from rmus_solution.msg import SquareArray
from pid import Pid

from rmus_solution.rmus_solution.scripts.manipulator import Manipulator

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

    def func(x=0, y=0, az=0):
        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = az
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        pub.publish(twist)
    return func


def identify_target_blocks():
    while not rospy.is_shutdown():
        sqrs: SquareArray = rospy.wait_for_message("/squares", SquareArray)
        high_sqrs = []
        for sqr in sqrs.data:
            h = sum([p.z for p in sqr.points])/4
            if h > 0.4:
                ya = sum([p.y for p in sqr.points])/4
                high_sqrs.append((sqr.id, ya))
        if len(high_sqrs) == 3:
            break
    high_sqrs.sort(key=lambda s: s[1], reverse=True)
    return [q[0] for q in high_sqrs]


def aim_block(set_speed: callable, block_id: int):
    pid_horizontal = Pid(-5, -0.01, 0)
    pid_depth = Pid(-1.5, -0.01, 0)
    r = rospy.Rate(20)
    lost = 0
    while not rospy.is_shutdown() and lost <= 5:
        sqrs: SquareArray = rospy.wait_for_message("/squares", SquareArray)
        target_sqrs = [sqr for sqr in sqrs.data if sqr.id == block_id]
        if not len(target_sqrs):
            rospy.loginfo(f"Square {block_id} not detected.")
            lost += 1
            continue
        else:
            lost = 0
        avg_h = 0
        avg_d = 0
        for sqr in target_sqrs:
            avg_h += sum([q.y for q in sqr.points])
            avg_d += sum([q.x for q in sqr.points])
        avg_h /= len(target_sqrs)*4
        avg_d /= len(target_sqrs)*4
        err_h = -0.02-avg_h
        err_d = 0.15-avg_d
        rospy.loginfo(f"ERR {err_d}, {err_h}")
        if abs(err_d) < 0.01 and abs(err_h) < 0.01:
            break
        speed_h = pid_horizontal(err_h)
        speed_d = pid_depth(err_d)
        # rospy.loginfo(f"SPEED {speed_d}, {speed_h}")
        set_speed(speed_d, speed_h)
        r.sleep()
    set_speed(0, 0)


def square_exists_in_view(id):
    sqrs: SquareArray = rospy.wait_for_message("/squares", SquareArray)
    for sqr in sqrs.data:
        if sqr.id == id:
            return True
    return False


if __name__ == '__main__':
    rospy.init_node("gamecore_node")

    set_speed = make_speed_setter("/cmd_vel")
    navi = Navi()
    manipulator = Manipulator()
    # navi.goto(PRE_DEFINED_POSE["noticeboard"])
    # targets = identify_target_blocks()
    # rospy.loginfo(f"TARGETS: {targets}")

    aim_block(set_speed, 2)
    rospy.sleep(5)
    manipulator.open_gripper()
    rospy.sleep(1)
    manipulator.down_arm()
    rospy.sleep(1)
    manipulator.close_gripper()
    rospy.sleep(1.0)
    manipulator.close_gripper()
    rospy.sleep(1.0)
    manipulator.reset_arm()
