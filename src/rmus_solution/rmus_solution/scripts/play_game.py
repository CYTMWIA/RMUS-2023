#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import inspect
import math
from functools import partial
from math import pi
from typing import List

import rospy
from geometry_msgs.msg import Twist
from manipulator import Manipulator
from navi import EpPose, Navi
from pid import Pid
from rmus_solution.msg import Square, SquareArray

# 预定义的路径点、名称: 位置(pose_x, pose_y, yaw), 误差容限(pose, angle)
PRE_DEFINED_POSE = {
    "home": EpPose(0.00, 0.00, 0.00, 0.02, 0.05),
    "noticeboard": EpPose(0.00, 1.60, 0.00),
    "station-1": EpPose(1.15, 1.91, 0.00, 0.075, 0.1),
    "station-2": EpPose(1.15, 1.80, 0.00, 0.075, 0.1),
    "station-3": EpPose(1.15, 1.65, 0.00, 0.075, 0.1),
    "patrol": [
        EpPose(1.15, 1.60, 0),
        EpPose(1.15, 1.60, pi/2),
        EpPose(1.15, 1.60, -pi/2),
        EpPose(0.00, 1.60, -pi/2),
        EpPose(0.00, 0.60, -pi/2),
        EpPose(0.80, 0.60, -pi/2),
        EpPose(0.80, -0.8, -pi/2),

        EpPose(2.7, -0.8, 0),  # 凸
        EpPose(1.8, -0.8, pi/2),
        EpPose(2.6, -0.8, pi/2),

        EpPose(2.55, 0.2, pi),
        EpPose(2.55, 0.2, pi/2),
        EpPose(2.55, 1.0, pi),
        EpPose(2.3, 1.0, pi/2),
        EpPose(2.3, 2.7, pi/2),

        EpPose(1.3, 2.7, pi),  # 左边区域
        EpPose(1.3, 3.3, 0),

        EpPose(0.8, 3.3, pi),  # 房间
        EpPose(0.3, 3.3, -pi/2),
        EpPose(0.0, 2.5, -pi/2),

        EpPose(0.0, 1.60, -pi/2),  # 回到观测点
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

########################################
# Square Functions
########################################


def get_squares_in_view() -> SquareArray:
    return rospy.wait_for_message("/squares", SquareArray)


def square_filter_by_id(sqrs: SquareArray, id_list: List[int]):
    return [sqr for sqr in sqrs.data if sqr.id in id_list]


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

########################################
# Behavior Core
########################################


class GameBehavior:
    def __init__(self) -> None:
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.navi = Navi(self.set_speed)
        self.manipulator = Manipulator()

        self.last_pose = PRE_DEFINED_POSE["noticeboard"]
        self.next_pose_list = []
        self.target_id_list = [0, 0, 0]

        rospy.sleep(1)
        rospy.loginfo("GameBehavior init done")

    ####################################
    # Functions
    ####################################

    def set_speed(self, x=0, y=0, az=0):
        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = az
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        self.pub_cmd_vel.publish(twist)

    def go_back(self, dis):
        self.set_speed(-dis*2, 0, 0)
        rospy.sleep(0.5)
        self.set_speed(0, 0, 0)

    def calc_block_center_point(self, sqrs: List[Square]):
        sum_x, sum_y, n_pts = 0, 0, 0
        for sqr in sqrs:
            sorted_by_z = sorted(sqr.points, key=lambda p: p.z)
            if sorted_by_z[-1].z-sorted_by_z[0].z < 2:
                sum_x += sum([p.x for p in sorted_by_z])/len(sorted_by_z)
                sum_y += sum([p.y for p in sorted_by_z])/len(sorted_by_z)
                n_pts += 1
            else:
                pt1, pt2 = sorted_by_z[0], sorted_by_z[1]
                if pt1.y == pt2.y:
                    continue
                elif pt1.x == pt2.x:
                    sum_x += pt1.x+0.0225
                    sum_y += (pt1.y + pt2.y)/2
                    n_pts += 1
                    continue
                if pt1.y < pt2.y: # make sure pt1.y > pt2.y
                    pt1, pt2 = pt2, pt1
                d = (pt1.x-pt2.x)/(pt1.y-pt2.y)
                dy = math.sqrt((0.0225**2)/(d*d+1))
                dx = d*dy
                if 0<d:
                    sum_y += -dy + (pt1.y+pt2.y)/2
                    sum_x += dx + (pt1.x+pt2.x)/2
                else:
                    sum_y += dy + (pt1.y+pt2.y)/2
                    sum_x += -dx + (pt1.x+pt2.x)/2
                n_pts += 1
        return sum_x/n_pts, sum_y/n_pts

    def aim_block(self, block_id: int, expect_x: float, expect_y: float):
        pid_y = Pid(-3.0, -0.020, 0)
        pid_x = Pid(-1.0, -0.020, 0)
        r = rospy.Rate(20)
        lost = 0
        while not rospy.is_shutdown() and lost <= 5:
            sqrs: SquareArray = get_squares_in_view()
            target_squares = square_filter_by_id(sqrs, [block_id])
            if not len(target_squares):
                rospy.loginfo(f"Square {block_id} not detected.")
                lost += 1
                continue
            else:
                lost = 0
            # avg_pts = [square_avg_point(s) for s in target_squares]
            # avg_x = sum([p.x for p in avg_pts])/len(avg_pts)
            # avg_y = sum([p.y for p in avg_pts])/len(avg_pts)
            avg_x, avg_y = self.calc_block_center_point(target_squares)
            err_y = expect_y-avg_y
            err_x = expect_x-avg_x
            # rospy.loginfo(f"ERR {err_x}, {err_y}")
            if abs(err_x) < 0.01 and abs(err_y) < 0.01:
                break
            speed_y = pid_y(err_y)
            speed_x = pid_x(err_x)
            # speed_yaw = pid_yaw(err_yaw)
            # rospy.loginfo(f"SPEED {speed_d}, {speed_h}")
            self.set_speed(speed_x, speed_y)
            r.sleep()
        self.set_speed(0, 0)
        return lost <= 5

    def find_reachable_work_pose(self, point):
        for r in [0, pi/4, pi*2/4, pi*3/4, pi]:
            xd = math.cos(r)*0.45
            yd = math.sin(r)*0.45
            pl = EpPose(point.x-xd, point.y+yd, -r,
                        frame="camera_aligned_depth_to_color_frame")
            pr = EpPose(pl.x, point.y-yd, r,
                        frame="camera_aligned_depth_to_color_frame")
            if self.navi.is_reachable(pr):
                return pr
            if self.navi.is_reachable(pl):
                return pl
        return None

    ####################################
    # Test
    ####################################

    def test_patrol(self):
        self.navi.goto_straight(PRE_DEFINED_POSE["noticeboard"])
        for pat in PRE_DEFINED_POSE["patrol"]:
            rospy.loginfo(f"Going {pat}")
            self.navi.goto_straight(pat)
            rospy.loginfo(f"Arrived {pat}")
        self.navi.goto(PRE_DEFINED_POSE["home"])

    def test_grab(self):
        block_id = 1
        aim_args_list = [
            (block_id, 0.16, -0.025, ),
            (block_id, 0.15, -0.025, ),
            (block_id, 0.13, -0.025, ),
            (block_id, 0.16, -0.025, ),
        ]
        for aim_args in aim_args_list:
            rospy.loginfo(f"Aim with args: {aim_args}")
            self.aim_block(*aim_args)
            self.manipulator.grab_block()

            rospy.sleep(1)
            self.go_back(0.08)

            confirm = get_squares_in_view()
            if len(square_filter_by_id(confirm, [block_id])):
                rospy.loginfo("Grab failed, try again")
            else:
                break
        self.manipulator.release_block()

    ####################################
    # Behavior (FSM)
    ####################################

    def end(self):
        rospy.loginfo(f"gg, let's go home")
        self.navi.goto(PRE_DEFINED_POSE["home"])
        return lambda: exit(0)

    def exchange_block(self, index: int):
        self.navi.goto(PRE_DEFINED_POSE[f"station-{index+1}"])
        self.aim_block(6+index, expect_x=0.16, expect_y=-0.04)
        self.manipulator.release_block()

        self.target_id_list[index] = 0
        self.go_back(0.15)  # avoid navi stuck
        if any(self.target_id_list):
            self.navi.goto(self.last_pose)
            return self.find_blocks
        else:
            return self.end

    def grab_block(self, work_pose: EpPose, block_id: int):
        self.navi.goto(work_pose)
        sqrs = get_squares_in_view()
        target_sqrs = square_filter_by_id(sqrs, [block_id])
        if len(target_sqrs):

            aim_args_list = [
                (block_id, 0.16, -0.025, ),
                (block_id, 0.15, -0.025, ),
                (block_id, 0.13, -0.025, ),
                (block_id, 0.16, -0.025, ),
            ]
            for aim_args in aim_args_list:
                rospy.loginfo(f"Aim with args: {aim_args}")
                self.aim_block(*aim_args)
                self.manipulator.grab_block()

                rospy.sleep(1)
                self.go_back(0.08)

                confirm = get_squares_in_view()
                if len(square_filter_by_id(confirm, [block_id])):
                    rospy.loginfo("Grab failed, try again")
                else:
                    return self.exchange_block(self.target_id_list.index(block_id))
            rospy.loginfo(
                "Grab failed too many times, will try again next time see it")

        else:
            rospy.loginfo("navi failed")

        # skip finding in current pose
        self.navi.goto(self.next_pose_list[0])
        self.last_pose = self.next_pose_list.pop(0)
        return self.find_blocks

    def find_blocks(self):
        if not len(self.next_pose_list):
            self.next_pose_list += PRE_DEFINED_POSE["patrol"].copy()
        rospy.loginfo(f"Finding: {self.target_id_list}")

        going_pose = self.next_pose_list[0]
        rotating = ((going_pose.x == self.last_pose.x)
                    and (going_pose.y == self.last_pose.y))
        self.navi.goto_straight(going_pose, blocking=rotating)

        while ((not rospy.is_shutdown())
               and ((not self.navi.is_finished())
                    or rotating)):
            sqrs = get_squares_in_view()
            target_sqrs = square_filter_by_id(sqrs, self.target_id_list)
            # rospy.loginfo(
            #     f"target squares in view {[s.id for s in target_sqrs]}")

            if len(target_sqrs):
                target_blocks = []
                for sqr in target_sqrs:
                    p_avg = square_avg_point(sqr)
                    dis = math.sqrt(p_avg.x*p_avg.x +
                                    p_avg.y*p_avg.y + p_avg.z*p_avg.z)
                    target_blocks.append((sqr.id, dis, p_avg, p_avg.z))
                target_blocks.sort(key=lambda b: b[1])

                for tid, dis, block_point, h in target_blocks:
                    work = self.find_reachable_work_pose(block_point)
                    if (1.5 < dis) or (0.4 < h) or (tid not in self.target_id_list):
                        continue
                    self.navi.pause()
                    rospy.loginfo(f"Block {tid} found, gogogo")
                    return lambda: self.grab_block(work, tid)

            if rotating:
                break

        self.last_pose = self.next_pose_list.pop(0)
        return self.find_blocks

    def begin(self):
        self.navi.goto_straight(PRE_DEFINED_POSE["noticeboard"])

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
            else:
                rospy.loginfo(f"target block ids not found")
        high_sqrs.sort(key=lambda s: s[1], reverse=True)
        self.target_id_list = [q[0] for q in high_sqrs]

        rospy.loginfo(f"Targets: {self.target_id_list}")
        return self.find_blocks

    def start_game(self):
        func = self.begin
        while not rospy.is_shutdown():
            label = func.__name__
            if label == "<lambda>":
                label = inspect.getsource(func)
                label = label[label.index("lambda"):].strip()

            rospy.loginfo(f"Enter -> {label}")
            func = func()
            rospy.loginfo(f"Leave <- {label}")


if __name__ == '__main__':
    rospy.init_node("gamecore_node")

    GameBehavior().start_game()
    # GameBehavior().test_patrol()
    # GameBehavior().test_grab()
