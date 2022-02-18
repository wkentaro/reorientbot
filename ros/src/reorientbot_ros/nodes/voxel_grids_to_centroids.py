#!/usr/bin/env python

import numpy as np

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from morefusion_ros.msg import ObjectPose
from morefusion_ros.msg import ObjectPoseArray
from morefusion_ros.msg import VoxelGridArray
import rospy
import topic_tools


class VoxelGridsToCentroids(topic_tools.LazyTransport):
    def __init__(self):
        super().__init__()
        self._pub_poses = self.advertise(
            "~output/poses", PoseArray, queue_size=1
        )
        self._pub_object_poses = self.advertise(
            "~output/object_poses", ObjectPoseArray, queue_size=1
        )
        self._post_init()

    def subscribe(self):
        self._sub = rospy.Subscriber(
            "~input",
            VoxelGridArray,
            self._callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

    def unsubscribe(self):
        self._sub.unregister()

    def _callback(self, grids_msg):
        object_poses_msg = ObjectPoseArray()
        object_poses_msg.header = grids_msg.header

        poses_msg = PoseArray()
        poses_msg.header = grids_msg.header

        for grid in grids_msg.grids:
            origin = np.array([grid.origin.x, grid.origin.y, grid.origin.z])
            indices = np.array(grid.indices)
            k = indices % grid.dims.z
            j = indices // grid.dims.z % grid.dims.y
            i = indices // grid.dims.z // grid.dims.y
            indices = np.column_stack((i, j, k))
            points = origin + indices * grid.pitch
            centroid = points.mean(axis=0)

            pose_msg = Pose()
            pose_msg.position.x = centroid[0]
            pose_msg.position.y = centroid[1]
            pose_msg.position.z = centroid[2]
            pose_msg.orientation.x = 0
            pose_msg.orientation.y = 0
            pose_msg.orientation.z = 0
            pose_msg.orientation.w = 1
            poses_msg.poses.append(pose_msg)

            object_pose_msg = ObjectPose()
            object_pose_msg.instance_id = grid.instance_id
            object_pose_msg.class_id = grid.class_id
            object_pose_msg.pose = pose_msg
            object_poses_msg.poses.append(object_pose_msg)

        self._pub_poses.publish(poses_msg)
        self._pub_object_poses.publish(object_poses_msg)


if __name__ == "__main__":
    rospy.init_node("voxel_grids_to_centroids")
    VoxelGridsToCentroids()
    rospy.spin()
