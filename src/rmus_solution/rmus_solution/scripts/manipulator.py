#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, Point

class Manipulator:
    def __init__(self) -> None:
        self.arm_gripper_pub = rospy.Publisher(
            "arm_gripper", Point, queue_size=2)
        self.arm_position_pub = rospy.Publisher(
            "arm_position", Pose, queue_size=2)

    def _open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        self.arm_gripper_pub.publish(open_gripper_msg)

    def _close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        self.arm_gripper_pub.publish(close_gripper_msg)

    def _reset_arm(self):
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        self.arm_position_pub.publish(reset_arm_msg)

    def _front_arm(self):
        pose = Pose()
        pose.position.x = 0.21
        pose.position.y = 0.0
        self.arm_position_pub.publish(pose)

    def _down_arm(self):
        pose = Pose()
        pose.position.x = 0.19
        pose.position.y = -0.08
        self.arm_position_pub.publish(pose)

    def grab_block(self):
        self._open_gripper()
        rospy.sleep(1)
        self._down_arm()
        rospy.sleep(1)
        self._close_gripper()
        rospy.sleep(1.0)
        self._reset_arm()

    def release_block(self):
        self._front_arm()
        rospy.sleep(1.0)
        self._open_gripper()
        rospy.sleep(1.0)
        self._reset_arm() 
