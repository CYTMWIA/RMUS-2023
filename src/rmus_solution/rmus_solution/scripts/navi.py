#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from threading import Thread

import rospy
import tf2_geometry_msgs
import tf2_ros
from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from pid import Pid
from tf_conversions import transformations

EpPose = namedtuple(
    "EpPose",
    field_names=["x", "y", "yaw", "err_position", "err_orientation", "frame"],
    defaults=[0, 0, 0, 0.1, 0.05, "map"]
)


def eppose2posestamped(eppose: EpPose) -> PoseStamped:
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = eppose.frame
    ps.pose.position.x = eppose.x
    ps.pose.position.y = eppose.y
    ps.pose.position.z = 0.0
    quat = transformations.quaternion_from_euler(0.0, 0.0, eppose.yaw)
    ps.pose.orientation.x = quat[0]
    ps.pose.orientation.y = quat[1]
    ps.pose.orientation.z = quat[2]
    ps.pose.orientation.w = quat[3]
    return ps


class Navi:
    Session = namedtuple(
        "Session", ["state", "target", "mode", "paused_point"])

    def __init__(self, set_speed: callable) -> None:
        self.set_speed = set_speed

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.thread: Thread = None

        self.move_base_client = SimpleActionClient(
            "/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

    def get_posestamped_in_map(self, eppose: EpPose) -> PoseStamped:
        if eppose.frame == "map":
            return eppose2posestamped(eppose)
        tran: TransformStamped = self.tf_buffer.lookup_transform(
            "map", eppose.frame, rospy.Time())
        return tf2_geometry_msgs.do_transform_pose(
            eppose2posestamped(eppose), tran)

    def is_reachable(self, eppose: EpPose) -> bool:
        ps = self.get_posestamped_in_map(eppose)
        res = self.make_plan(
            self.get_posestamped_in_map(EpPose(0, 0, 0, frame="base_link")),
            ps,
            0
        )
        # rospy.loginfo(res.plan.poses[-1])
        # rospy.loginfo(ps)
        return len(res.plan.poses) > 0

    def goto(self, pose: EpPose, blocking=True):
        rospy.loginfo(f"GOTO {pose}")

        simple_goal = MoveBaseGoal()
        if isinstance(pose, EpPose):
            simple_goal.target_pose = self.get_posestamped_in_map(pose)
        else:
            simple_goal.target_pose = pose

        if blocking:
            res = self.move_base_client.send_goal_and_wait(simple_goal)
            return res
        else:
            res = self.move_base_client.send_goal(simple_goal)
            return res

    def _get_pose_in_base_link(self, eppose: EpPose):
        tran = self.tf_buffer.lookup_transform(
            'base_link', 'map', rospy.Time())
        pose_in_base_link = tf2_geometry_msgs.do_transform_pose(
            eppose2posestamped(eppose), tran)
        euler = transformations.euler_from_quaternion([
            pose_in_base_link.pose.orientation.x,
            pose_in_base_link.pose.orientation.y,
            pose_in_base_link.pose.orientation.z,
            pose_in_base_link.pose.orientation.w,
        ])
        return EpPose(
            pose_in_base_link.pose.position.x,
            pose_in_base_link.pose.position.y,
            euler[2],
            eppose.err_position,
            eppose.err_orientation,
            "base_link"
        )

    def is_finished(self):
        return self.move_base_client.get_state() >= 3

    def cancel(self):
        self.move_base_client.cancel_all_goals()
        while self.move_base_client.get_state() == GoalStatus.ACTIVE:
            rospy.sleep(0.01)
