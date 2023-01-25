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
        self.state = "init"
        self.last_goal = None
        self.last_paused_pose = None

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
        self.state = "running"
        rospy.loginfo(f"GOTO {pose}")
        self._send_goal(pose)
        self.last_goal = pose
        if blocking:
            self.move_base_client.wait_for_result()
            self.state = "finished"
            return self.move_base_client.get_result()
        else:
            return None

    def is_finished(self):
        if self.move_base_client.get_state() == 3:
            self.state = "finished"
        return self.state == "finished"

    def pause(self):
        self.state = "paused"
        self.move_base_client.cancel_all_goals()
        # TODO: remember current position
        pass

    def resume(self):
        self.state = "running"
        if self.last_paused_pose != None:
            self.goto(self.last_paused_pose)
            self.last_paused_pose = None
        self.goto(self.last_goal)
