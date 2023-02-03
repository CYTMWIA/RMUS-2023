#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import inspect
import math
from functools import partial
from math import pi
from typing import List

import rospy
from actionlib_msgs.msg import GoalStatus as NaviStatus
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
        EpPose(0.00, 1.60, 0.00),
        EpPose(0.80, 0.80, 0.00),
        EpPose(2.30, -0.6, 0),
        EpPose(2.30, 1.00, 0),
        EpPose(1.20, 3.00, 0),
        EpPose(0.55, 3.20, 0),
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
    x_min = min([pt.x for pt in sqr.quads])
    x_max = max([pt.x for pt in sqr.quads])
    y_min = min([pt.y for pt in sqr.quads])
    y_max = max([pt.y for pt in sqr.quads])
    return (x_min, y_min, x_max-x_min, y_max-y_min,)


def square_bbox_area(sqr: Square):
    x, y, w, h = square_bbox(sqr)
    return w*h


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
        sqrs = sorted(sqrs.copy(), key=square_bbox_area, reverse=True)
        for sqr in sqrs:
            sorted_by_z = sorted(sqr.points, key=lambda p: p.z)
            if sorted_by_z[-1].z-sorted_by_z[0].z < 0.02:
                return sum([p.x for p in sorted_by_z])/len(sorted_by_z), sum([p.y for p in sorted_by_z])/len(sorted_by_z)
            else:
                pt1, pt2 = sorted_by_z[-1], sorted_by_z[-2]
                if pt1.y == pt2.y:
                    continue
                elif pt1.x == pt2.x:
                    return pt1.x+0.025, (pt1.y + pt2.y)/2
                if pt1.x > pt2.x:  # make sure pt1.x < pt2.x
                    pt1, pt2 = pt2, pt1
                d = (pt2.y-pt1.y)/(pt2.x-pt1.x)
                rad = math.atan(-1/d)
                return (pt1.x+pt2.x)/2 + math.cos(rad)*0.025, (pt1.y+pt2.y)/2 + math.sin(rad)*0.025,
        return None

    def calc_block_round_poses(self, sqrs: List[Square], dis: float):
        for sqr in sqrs:
            sorted_by_z = sorted(sqr.points, key=lambda p: p.z)
            if sorted_by_z[-1].z-sorted_by_z[0].z < 0.02:
                continue
            else:
                pt1, pt2 = sorted_by_z[-1], sorted_by_z[-2]
                if pt1.y == pt2.y:
                    continue
                elif pt1.x == pt2.x:
                    cx, cy = (pt1.x+pt2.x)/2+0.025, (pt1.y+pt2.y)/2
                    return [EpPose(cx-dis, cy, 0,
                                   frame="camera_aligned_depth_to_color_frame"),
                            EpPose(cx+dis, cy, pi,
                                   frame="camera_aligned_depth_to_color_frame"),
                            EpPose(cx, cy-dis, pi/2,
                                   frame="camera_aligned_depth_to_color_frame"),
                            EpPose(cx, cy+dis, -pi/2,
                                   frame="camera_aligned_depth_to_color_frame"),]

                if pt1.x > pt2.x:  # make sure pt1.x < pt2.x
                    pt1, pt2 = pt2, pt1
                d = (pt2.y-pt1.y)/(pt2.x-pt1.x)
                dx = math.sqrt((0.025**2)/(d**2+1))
                dy = d*dx
                cx, cy = (pt1.x+pt2.x)/2+dx, (pt1.y+pt2.y)/2+dy
                res = []
                for r in [0, pi/2, pi, -pi/2]:
                    rad = math.atan(d) + r
                    dy = math.sin(rad)*dis
                    dx = math.cos(rad)*dis
                    res.append(EpPose(cx + dx, cy + dy, pi+rad,
                                      frame="camera_aligned_depth_to_color_frame"))
                return res
        return []

    def aim_block(self, block_id: int, expect_x: float, expect_y: float, toleration_x: float, toleration_y: float):
        pid_y = Pid(-5.0, -0.02, 0)
        pid_x = Pid(-1.5, -0.02, 0)
        r = rospy.Rate(20)
        lost = 0
        stage = 0
        while not rospy.is_shutdown() and lost <= 5:
            sqrs: SquareArray = get_squares_in_view()
            target_squares = square_filter_by_id(sqrs, [block_id])
            if not len(target_squares):
                rospy.loginfo(f"Square {block_id} not detected.")
                lost += 1
                speed_x, speed_y = 0, 0
            else:
                lost = 0
                avg_x, avg_y = self.calc_block_center_point(target_squares)
                err_y = expect_y-avg_y
                err_x = expect_x-avg_x
                # rospy.loginfo(f"POS {avg_x}, {avg_y}")
                # rospy.loginfo(f"ERR {err_x}, {err_y}")
                if stage == 0:
                    speed_y = pid_y(err_y)
                    speed_x = 0
                    if abs(err_y) < toleration_y*2:
                        stage += 1
                elif stage == 1:
                    speed_y = pid_y(err_y)
                    speed_x = pid_x(err_x)
                    if abs(err_x) < toleration_x and abs(err_y) < toleration_y:
                        break
            # rospy.loginfo(f"SPEED {speed_d}, {speed_h}")
            self.set_speed(speed_x, speed_y)
            r.sleep()
        self.set_speed(0, 0)
        return lost <= 5

    def find_reachable_grabbing_poses(self, sqrs: List[Square]):
        poses = self.calc_block_round_poses(sqrs, 0.5)
        poses.sort(key=lambda p: p.x**2+p.y**2)
        return [self.navi.get_posestamped_in_map(po) for po in poses if self.navi.is_reachable(po)]

    def detect_target_blocks(self):
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
                if (2.0 < dis) or (0.4 < h) or (tid not in self.target_id_list):
                    continue
                rospy.loginfo(f"Block {tid} found, gogogo")
                return tid
        else:
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

    ####################################
    # Behavior (FSM)
    ####################################

    def end(self):
        rospy.loginfo(f"gg, let's go home")
        self.navi.goto(PRE_DEFINED_POSE["home"])
        return None

    def exchange_block(self, index: int):
        while self.navi.goto(PRE_DEFINED_POSE[f"station-{index+1}"])!=NaviStatus.SUCCEEDED:
            self.go_back(0.2)

        self.aim_block(6+index,
                       expect_x=0.16, expect_y=-0.04,
                       toleration_x=0.01, toleration_y=0.01)
        self.manipulator.release_block()

        self.target_id_list[index] = 0
        self.go_back(0.15)  # avoid navi stuck
        if any(self.target_id_list):
            return self.find_blocks_goto
        else:
            return self.end

    def grab_block(self, block_id: int):
        aim_args_list = [
            (block_id, 0.16, -0.03, 0.01, 0.005),
            (block_id, 0.15, -0.03, 0.01, 0.005),
            (block_id, 0.17, -0.03, 0.01, 0.005),
            (block_id, 0.16, -0.03, 0.01, 0.005),
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
                return lambda: self.exchange_block(self.target_id_list.index(block_id))

        rospy.loginfo(
            "Grab failed too many times, will try again next time see it")
        # skip finding in current pose
        self.navi.goto(self.next_pose_list[0])
        self.last_pose = self.next_pose_list.pop(0)
        return self.find_blocks_goto

    def goto_grabbing_pose(self, block_id: int):
        sqrs = get_squares_in_view()
        target_sqrs = square_filter_by_id(sqrs, [block_id])
        poses = self.find_reachable_grabbing_poses(target_sqrs)
        rospy.loginfo(f"Found work pose candidates: {poses}")
        for ps in poses:
            nv = self.navi.goto(ps)
            if nv!=NaviStatus.SUCCEEDED:
                rospy.loginfo("goto grabing pose failed, try next one")
                continue

            sqrs = get_squares_in_view()
            target_sqrs = square_filter_by_id(sqrs, [block_id])
            if len(target_sqrs):
                return lambda: self.grab_block(block_id)

        rospy.loginfo("goto_grabbing_pose all failed!")
        # skip finding in current pose
        self.navi.goto(self.next_pose_list[0])
        self.last_pose = self.next_pose_list.pop(0)
        return self.find_blocks_goto

    def find_blocks_rotate(self):
        t = 15
        az = 2*pi/t
        self.set_speed(0, 0, az)
        end = rospy.get_time() + t

        while (not rospy.is_shutdown()) and (rospy.get_time() < end):
            tid = self.detect_target_blocks()
            if tid:
                self.set_speed(0, 0, 0)
                return lambda: self.goto_grabbing_pose(tid)

        self.set_speed(0, 0, 0)
        self.last_pose = self.next_pose_list.pop(0)
        return self.find_blocks_goto

    def find_blocks_goto(self):
        if not len(self.next_pose_list):
            self.next_pose_list += PRE_DEFINED_POSE["patrol"].copy()
        rospy.loginfo(f"Finding: {self.target_id_list}")

        going_pose = self.next_pose_list[0]
        self.navi.goto(going_pose, blocking=False)

        while (not rospy.is_shutdown()) and (not self.navi.is_finished()):
            tid = self.detect_target_blocks()
            if tid:
                self.navi.cancel()
                return lambda: self.goto_grabbing_pose(tid)

        return self.find_blocks_rotate

    def begin(self):
        while self.navi.goto(PRE_DEFINED_POSE["noticeboard"]) != NaviStatus.SUCCEEDED:
            self.navi.goto(PRE_DEFINED_POSE["home"])

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
        return self.find_blocks_goto

    def start_game(self):
        func = self.begin
        while not rospy.is_shutdown():
            if func == None:
                break

            label = func.__name__
            if label == "<lambda>":
                label = inspect.getsource(func)
                label = label[label.index("lambda"):].strip()

            rospy.loginfo(f"Enter -> {label}")
            func = func()
            rospy.loginfo(f"Leave <- {label}")


if __name__ == '__main__':
    rospy.init_node("gamecore_node")

    gb = GameBehavior()
    gb.start_game()

    # gb.test_patrol()

    # bid = 4
    # # gb.goto_grab_block(bid)
    # gb.grab_block(bid)
    # gb.manipulator.release_block()
    # gb.end()
