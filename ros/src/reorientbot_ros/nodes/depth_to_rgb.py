#!/usr/bin/env python

import imgviz
import numpy as np

import cv_bridge
import message_filters
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import topic_tools

import reorientbot


class DepthToNormalNode(topic_tools.LazyTransport):
    def __init__(self):
        super().__init__()
        self._pub_normal = self.advertise(
            "~output/normal", Image, queue_size=1
        )
        self._pub_jet = self.advertise("~output/jet", Image, queue_size=1)
        self._post_init()

    def subscribe(self):
        sub_cam = message_filters.Subscriber(
            "~input/camera_info", CameraInfo, queue_size=1
        )
        sub_depth = message_filters.Subscriber(
            "~input/depth", Image, queue_size=1
        )
        self._subscribers = [sub_cam, sub_depth]
        sync = message_filters.TimeSynchronizer(
            self._subscribers, queue_size=5
        )
        sync.registerCallback(self._callback)

    def unsubscribe(self):
        for sub in self._subscribers:
            sub.unregister()

    def _callback(self, cam_msg, depth_msg):
        bridge = cv_bridge.CvBridge()

        depth = bridge.imgmsg_to_cv2(depth_msg)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000
            depth[depth == 0] = np.nan
        assert depth.dtype == np.float32

        if self._pub_normal.get_num_connections() > 0:
            K = np.array(cam_msg.K).reshape(3, 3)
            points = reorientbot.geometry.pointcloud_from_depth(
                depth, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
            )
            normal = reorientbot.geometry.normals_from_pointcloud(points)
            normal = np.uint8((normal + 1) / 2 * 255)
            out_msg = bridge.cv2_to_imgmsg(normal, "rgb8")
            out_msg.header = cam_msg.header
            self._pub_normal.publish(out_msg)

        if self._pub_jet.get_num_connections() > 0:
            jet = imgviz.depth2rgb(depth, min_value=0.3, max_value=1)
            out_msg = bridge.cv2_to_imgmsg(jet, "rgb8")
            out_msg.header = cam_msg.header
            self._pub_jet.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("depth_to_normal")
    DepthToNormalNode()
    rospy.spin()
