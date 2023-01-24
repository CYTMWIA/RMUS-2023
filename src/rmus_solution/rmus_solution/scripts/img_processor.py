#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from threading import Thread

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from detect import load_template, marker_detection
from geometry_msgs.msg import Point
from rmus_solution.msg import Square, SquareArray
from sensor_msgs.msg import CameraInfo, Image


class ImgProcessor:
    def __init__(self, verbose=True) -> None:
        self.bridge = CvBridge()

        self.camera_info = rospy.wait_for_message(
            "/camera/color/camera_info", CameraInfo, timeout=5.0)
        self.camera_matrix = np.array(
            self.camera_info.K, "double").reshape((3, 3))

        self.sub_image = message_filters.Subscriber(
            "/camera/color/image_raw", Image)
        self.sub_depth = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image)
        self.sync_image = message_filters.TimeSynchronizer(
            [self.sub_image, self.sub_depth], 10)
        self.sync_image.registerCallback(self.image_callback)

        self.pub_squares = rospy.Publisher(
            "/squares", SquareArray, queue_size=1)

        if verbose:
            self.vis_thread = Thread(target=self.visualization)
            self.vis_thread.start()

    def calc_space_point(self, img_x, img_y):
        p = Point()
        fx = self.camera_matrix[0][0]
        fy = self.camera_matrix[1][1]
        cx = self.camera_matrix[0][2]
        cy = self.camera_matrix[1][2]
        p.x = self.depth[img_y][img_x]
        p.y = (cx - img_x) * p.x/fx
        p.z = (cy - img_y) * p.x/fy
        return p

    def visualization(self):
        while not rospy.is_shutdown():
            try:
                cv2.imshow("frame", self.image)
                cv2.waitKey(1)
            except:
                rospy.sleep(1)

    def image_callback(self, image, depth):
        self.image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.depth = self.bridge.imgmsg_to_cv2(depth, "32FC1")

        block_ids, block_quads, \
            *_ = marker_detection(self.image, self.camera_matrix)
        # block_quads =
        block_quad_points = [
            [self.calc_space_point(*q) for q in quads]
            for quads in block_quads
        ]

        squares = SquareArray()
        squares.header.stamp = rospy.Time.now()
        for i in range(len(block_ids)):
            if block_ids[i] == -1:
                continue
            sq = Square()
            sq.id = block_ids[i]
            sq.quads = []
            for quad in block_quads[i]:
                sq.quads.append(Point())
                sq.quads[-1].x = quad[0]
                sq.quads[-1].y = quad[1]
            sq.points = block_quad_points[i]
            squares.data.append(sq)
        self.pub_squares.publish(squares)


if __name__ == "__main__":
    rospy.init_node("image_node", anonymous=True)
    load_template()
    img_proc = ImgProcessor(verbose=False)
    rospy.loginfo("Image thread started")
    rospy.spin()
