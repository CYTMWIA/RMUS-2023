#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8MultiArray, Int32MultiArray
from rmus_solution.srv import switch, setgoal, graspsignal
from typing import List
from functools import partial
from navi_control import Navi

def get_boxid_blockid_inorder(gameinfo):
    return [0,1,2], [gameinfo[0], gameinfo[1], gameinfo[2]]

def wait_for_services(services: List[str]):
    for ser in services:
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(ser, 1.0)
                break
            except:
                rospy.logwarn(f"Waiting for '{ser}' Service")
                rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node("gamecore_node")

    wait_for_services([
        "/set_navigation_goal", 
        "/let_manipulater_work", 
        "/image_processor_switch_mode"
    ])
    rospy.loginfo("Get all rospy sevice!")

    navigation = rospy.ServiceProxy("/set_navigation_goal", setgoal)
    navi = lambda point_name: navigation(Navi.point(point_name), point_name)
    trimer = rospy.ServiceProxy("/let_manipulater_work", graspsignal)
    img_switch_mode = rospy.ServiceProxy("/image_processor_switch_mode", switch)
    rospy.sleep(2)

    trim_res = trimer(0, "")
    response = img_switch_mode(9)
    navigation_result = navi("noticeboard")

    gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)
    while gameinfo.data[0] == 0 or gameinfo.data[1] == 0 or gameinfo.data[2] == 0:
       gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)
       rospy.logwarn("Waiting for gameinfo detection.")
       rospy.sleep(0.5)

    response = img_switch_mode(0)
    
    for i, target in enumerate(gameinfo.data):
        navigation_result = navigation(target, "")

        response = img_switch_mode(target)
        rospy.sleep(0.5)
        trimer_response = trimer(1,"")

        try:
            rospy.sleep(0.5)
            blockinfo = rospy.wait_for_message("/get_blockinfo", Int32MultiArray, timeout=0.1)
            rospy.sleep(0.5)
            blockinfo2 = rospy.wait_for_message("/get_blockinfo", Int32MultiArray, timeout=0.1)

            if blockinfo.data[-1] != blockinfo2.data[-1]:
                rospy.logwarn("Catch Falied")
                rospy.sleep(3.0)
                trimer_response = trimer(1,"")
                while trimer_response.res == False:
                    trimer_response = trimer(1,"")
                rospy.loginfo("Another Catch completed")
            else:
                rospy.loginfo("Catch success")
        except:
            rospy.logerr("Catch possible error")

        response = img_switch_mode(0)
        navigation_result = navigation(6+i, "")

        response = img_switch_mode(6+i)
        rospy.sleep(0.5)
        trimer_response = trimer(2,"")
        response = img_switch_mode(0)

    navigation_result = navigation(0, "")
    response = img_switch_mode(0)
