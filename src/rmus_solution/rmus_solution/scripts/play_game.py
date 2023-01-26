#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from math import pi
from functools import partial
from typing import List

import rospy
from geometry_msgs.msg import Twist
from manipulator import Manipulator
from navi import EpPose, Navi
from pid import Pid
from rmus_solution.msg import SquareArray, Square

# 预定义的路径点、名称: 位置(pose_x, pose_y, yaw), 误差容限(pose, angle)
PRE_DEFINED_POSE = {
    "home": EpPose(0.00, 0.00, 0.00, 0.02, 0.05),
    "noticeboard": EpPose(0.00, 1.60, 0.00),
    "station-1": EpPose(1.15, 1.91, 0.00, 0.075, 0.1),
    "station-2": EpPose(1.15, 1.80, 0.00, 0.075, 0.1),
    "station-3": EpPose(1.15, 1.65, 0.00, 0.075, 0.1),
    "patrol": [
        EpPose(0.00, 1.60, -pi/2),
        EpPose(0.00, 1.60, -pi/3),
        EpPose(0, 0, -pi/4),
        EpPose(1.0, -0.4, -pi/3),
        EpPose(1.5, -0.7, 0),
        EpPose(2.3, -0.7, 0),
        EpPose(2.3, -0.7, pi/2),
        EpPose(2.3, 2.3, pi/2),
        EpPose(2.3, 2.3, pi*2/3),
        EpPose(2.3, 2.5, pi),
        EpPose(1.8, 3.4, pi),
        EpPose(1.3, 3.4, -pi/2),
        EpPose(1.0, 3.0, pi),
        EpPose(0.6, 3.4, -pi/2),
        EpPose(0.0, 2.3, -pi/2),
    ],
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


def get_squares_in_view() -> SquareArray:
    return rospy.wait_for_message("/squares", SquareArray)


def identify_target_blocks():
    while not rospy.is_shutdown():
        sqrs: SquareArray = get_squares_in_view()
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


def square_filter_by_id(sqrs: SquareArray, id_list: List[int]):
    return [sqr for sqr in sqrs.data if sqr.id in id_list]


def aim_block(set_speed: callable, block_id: int, expect_x=0.16, expect_y=-0.02):
    pid_y = Pid(-3.0, -0.020, 0)
    pid_x = Pid(-1.0, -0.020, 0)
    r = rospy.Rate(20)
    lost = 0
    while not rospy.is_shutdown() and lost <= 5:
        sqrs: SquareArray = get_squares_in_view()
        target_sqrs = square_filter_by_id(sqrs, [block_id])
        if not len(target_sqrs):
            rospy.loginfo(f"Square {block_id} not detected.")
            lost += 1
            continue
        else:
            lost = 0
        target_sqrs = [(s, square_bbox(s)) for s in target_sqrs]
        target_sqrs.sort(key=lambda s: s[1][1][0]*s[1][1][1], reverse=True)
        avg_pt = square_avg_point(target_sqrs[0][0])
        err_y = expect_y-avg_pt.y
        err_x = expect_x-avg_pt.x
        rospy.loginfo(f"ERR {err_x}, {err_y}")
        if abs(err_x) < 0.01 and abs(err_y) < 0.01:
            break
        speed_y = pid_y(err_y)
        speed_x = pid_x(err_x)
        # speed_yaw = pid_yaw(err_yaw)
        # rospy.loginfo(f"SPEED {speed_d}, {speed_h}")
        set_speed(speed_x, speed_y)
        r.sleep()
    set_speed(0, 0)
    return lost <= 5


def square_bbox(sqr: Square):
    x_min = min([pt.x for pt in sqr.points])
    x_max = max([pt.x for pt in sqr.points])
    y_min = min([pt.y for pt in sqr.points])
    y_max = max([pt.y for pt in sqr.points])
    return ((x_min, y_min,), (x_max-x_min, y_max-y_min,), )


def square_avg_point(sqr: Square):
    pt = sqr.points[0]
    for i in range(1, len(sqr.points)):
        pt.x += sqr.points[i].x
        pt.y += sqr.points[i].y
        pt.z += sqr.points[i].z
    pt.x /= len(sqr.points)
    pt.y /= len(sqr.points)
    pt.z /= len(sqr.points)
    return pt


def find_reachable_near_point(navi: Navi, point):
    for r in [0, pi/4, pi*2/4, pi*3/4, pi]:
        xd = math.cos(r)*0.3
        yd = math.sin(r)*0.3
        p1 = EpPose(point.x-xd, point.y+yd, -r)
        p2 = EpPose(p1.x, point.y-yd, r)
        if navi.is_reachable(p1, "base_link"):
            return p1
        if navi.is_reachable(p2, "base_link"):
            return p2
    return None


if __name__ == '__main__':
    rospy.init_node("gamecore_node")

    set_speed = make_speed_setter("/cmd_vel")
    navi = Navi(set_speed)
    manipulator = Manipulator()
    rospy.sleep(1)
    rospy.loginfo("Init done.")

    ####################################
    # temporary debug
    ####################################
    aim_block(set_speed, 4)
    manipulator.grab_block()
    manipulator.release_block()
    exit(0)
    ####################################

    navi.goto_straight(PRE_DEFINED_POSE["noticeboard"])
    target_ids = identify_target_blocks()
    rospy.loginfo(f"TARGETS: {target_ids}")

    while not rospy.is_shutdown():
        last_pat = None
        for pat in PRE_DEFINED_POSE["patrol"]:
            navi.goto_straight(pat, blocking=False)
            while (not rospy.is_shutdown()) and (not navi.is_finished()):
                sqrs = get_squares_in_view()
                target_sqrs = square_filter_by_id(sqrs, target_ids)
                rospy.loginfo(
                    f"target squares in view {[s.id for s in target_sqrs]}")

                target_block_id = None
                target_work_point = None
                if len(target_sqrs):
                    target_blocks = []
                    for sqr in target_sqrs:
                        p_avg = square_avg_point(sqr)
                        dis = math.sqrt(p_avg.x*p_avg.x +
                                        p_avg.y*p_avg.y + p_avg.z*p_avg.z)
                        target_blocks.append((sqr.id, dis, p_avg))
                    target_blocks.sort(key=lambda b: b[1])

                    target_block_id = None
                    target_work_point = None
                    for tid, dis, block_point in target_blocks:
                        work = find_reachable_near_point(navi, block_point)
                        if (1 < dis) or (tid not in target_ids):
                            continue
                        target_block_id = tid
                        target_work_point = work
                        break

                if target_block_id != None:
                    rospy.loginfo(f"Working on block {target_block_id}")
                    navi.pause()
                    # navi.goto(target_work_point, "base_link")
                    while True:
                        aim_block(set_speed, tid)
                        manipulator.grab_block()
                        rospy.sleep(2)
                        set_speed(-0.15, 0)
                        rospy.sleep(0.5)
                        set_speed(0, 0)
                        confirm = get_squares_in_view()
                        if len(square_filter_by_id(confirm, [tid])):
                            rospy.loginfo("Grab failed")
                        else:
                            break
                    target_idx = target_ids.index(tid)
                    target_ids[target_idx] = 0
                    navi.goto(PRE_DEFINED_POSE[f"station-{target_idx+1}"])
                    aim_block(set_speed, 6+target_idx, expect_y=-0.04)
                    manipulator.release_block()

                    rospy.loginfo(f"Back to last pat")
                    if last_pat == None:
                        navi.goto(PRE_DEFINED_POSE["noticeboard"])
                    else:
                        navi.goto(last_pat)

                if not any(target_ids):
                    navi.goto(PRE_DEFINED_POSE["home"])
                    exit(0)
            last_pat = pat

    navi.goto(PRE_DEFINED_POSE["home"])
