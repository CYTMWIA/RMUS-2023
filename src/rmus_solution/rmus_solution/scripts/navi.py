#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import namedtuple

import rospy
from actionlib.simple_action_client import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf_conversions import transformations

EpPose = namedtuple(
    "EpPose",
    field_names=["x", "y", "yaw", "err_position", "err_angle"],
    defaults=[0, 0, 0, 0, 0]
)


class Navi:
    def __init__(self) -> None:
        self.move_base_client = SimpleActionClient(
            "/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()

    def _send_goal(self, pose: EpPose):
        simple_goal = MoveBaseGoal()
        simple_goal.target_pose.header.stamp = rospy.Time.now()
        simple_goal.target_pose.header.frame_id = "map"
        simple_goal.target_pose.pose.position.x = pose.x
        simple_goal.target_pose.pose.position.y = pose.y
        simple_goal.target_pose.pose.position.z = 0.0
        quat = transformations.quaternion_from_euler(0.0, 0.0, pose.yaw)
        simple_goal.target_pose.pose.orientation.x = quat[0]
        simple_goal.target_pose.pose.orientation.y = quat[1]
        simple_goal.target_pose.pose.orientation.z = quat[2]
        simple_goal.target_pose.pose.orientation.w = quat[3]
        self.move_base_client.send_goal(simple_goal)

    def goto(self, pose: EpPose, blocking=True):
        rospy.loginfo(f"GOTO {pose}")
        self._send_goal(pose)
        if blocking:
            self.move_base_client.wait_for_result()
            return self.move_base_client.get_result()
        else:
            return None
