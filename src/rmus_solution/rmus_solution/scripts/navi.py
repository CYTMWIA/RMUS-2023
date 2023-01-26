#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from threading import Thread
import rospy
import tf2_geometry_msgs
import tf2_ros
from actionlib.simple_action_client import SimpleActionClient
from geometry_msgs.msg import PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pid import Pid
from tf_conversions import transformations
from nav_msgs.srv import GetPlan

EpPose = namedtuple(
    "EpPose",
    field_names=["x", "y", "yaw", "err_position", "err_orientation"],
    defaults=[0, 0, 0, 0.1, 0.05]
)


class Navi:
    Session = namedtuple(
        "Session", ["state", "target", "mode", "paused_point"])

    def __init__(self, set_speed: callable) -> None:
        self.set_speed = set_speed

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.session = []
        self.thread: Thread = None

        self.move_base_client = SimpleActionClient(
            "/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

    def get_ros_pose_in_map(self, base_link_eppose:EpPose):
        tran: TransformStamped = self.tf_buffer.lookup_transform(
            'map', 'base_link', rospy.Time())
        return tf2_geometry_msgs.do_transform_pose(
            self._make_ros_pose(base_link_eppose, "base_link"), tran)

    def is_reachable(self, pose: EpPose, frame="map"):
        if frame=="map":
            ps = self._make_ros_pose(pose, frame)
        elif frame=="base_link":
            ps = self.get_ros_pose_in_map(pose)
        res = self.make_plan(
            self.get_ros_pose_in_map(EpPose(0,0,0)),
            ps,
            0.01
        )
        return len(res.plan.poses) > 0

    def _make_ros_pose(self, eppose: EpPose, frame: str):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = frame
        ps.pose.position.x = eppose.x
        ps.pose.position.y = eppose.y
        ps.pose.position.z = 0.0
        quat = transformations.quaternion_from_euler(0.0, 0.0, eppose.yaw)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        return ps

    def _send_goal(self, pose: EpPose, frame: str):
        simple_goal = MoveBaseGoal()
        if frame=="map":
            ps = self._make_ros_pose(pose, frame)
        elif frame=="base_link":
            ps = self.get_ros_pose_in_map(pose)
        simple_goal.target_pose = ps
        self.move_base_client.send_goal(simple_goal)

    def goto(self, pose: EpPose, blocking=True, frame="map"):
        self.session.append(self.Session("running", pose, "navi", None))
        rospy.loginfo(f"GOTO {pose}")
        self._send_goal(pose, frame)
        self.last_goal = pose
        if blocking:
            self.move_base_client.wait_for_result()
            self.session[-1] = self.session[-1]._replace(state="finished")
            return self.move_base_client.get_result()
        else:
            return None

    def _get_pose_in_base_link(self, eppose: EpPose):
        tran = self.tf_buffer.lookup_transform(
            'base_link', 'map', rospy.Time())
        pose_in_base_link = tf2_geometry_msgs.do_transform_pose(
            self._make_ros_pose(eppose, "map"), tran)
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
            eppose.err_orientation
        )

    def _goto_straight(self, pose: EpPose):
        rospy.loginfo(f"GOTO STRAIGHT {pose}")
        r = rospy.Rate(20)
        pid_p = Pid(3, 0.2, 0)
        pid_r = Pid(5, 0.01, 0)
        while self.session[-1].state == "running" and not rospy.is_shutdown():
            target = self._get_pose_in_base_link(pose)
            err = target.yaw
            if abs(err) < pose.err_orientation:
                self.set_speed(0, 0, 0)
                break
            else:
                self.set_speed(0, 0, pid_r(err))
            r.sleep()
        while self.session[-1].state == "running" and not rospy.is_shutdown():
            target = self._get_pose_in_base_link(pose)
            err = math.sqrt(target.x*target.x + target.y*target.y)
            if abs(err) < pose.err_position:
                self.set_speed(0, 0, 0)
                break
            else:
                p = pid_p(err)
                self.set_speed(target.x*p, target.y*p, 0)
            r.sleep()
        finished = False
        while self.session[-1].state == "running" and not rospy.is_shutdown():
            target = self._get_pose_in_base_link(pose)
            err = target.yaw
            if abs(err) < pose.err_orientation:
                finished = True
                break
            else:
                self.set_speed(0, 0, pid_r(err))
            r.sleep()
        self.set_speed(0, 0, 0)
        if finished:
            self.session[-1] = self.session[-1]._replace(state="finished")

    def goto_straight(self, pose: EpPose, blocking=True):
        self.session.append(self.Session("running", pose, "straight", None))
        if blocking:
            self._goto_straight(pose)
        else:
            self.thread = Thread(target=self._goto_straight, args=[pose])
            self.thread.start()

    def is_finished(self):
        if len(self.session) == 0:
            return False
        if self.session[-1].mode == "navi":
            if self.move_base_client.get_state() == 3:
                self.session[-1] = self.session[-1]._replace(state="finished")
        return self.session[-1].state == "finished"

    def pause(self):
        if len(self.session) == 0:
            return
        self.session[-1] = self.session[-1]._replace(state="paused")
        self.move_base_client.cancel_all_goals()
        if self.thread != None and self.thread.isAlive():
            self.thread.join()
        # TODO: remember current position
        pass

    def resume(self):
        while self.session[-1].state == "finished":
            self.session.pop()
        if len(self.session) == 0:
            return
        self.session[-1] = self.session[-1]._replace(state="running")
        if self.session[-1].paused_point != None:
            self.goto(self.session[-1].paused_point)
            self.session[-1] = self.session[-1]._replace(paused_point=None)
        self.goto(self.last_goal)
