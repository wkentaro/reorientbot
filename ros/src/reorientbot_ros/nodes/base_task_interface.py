#!/usr/bin/env python

import argparse
import time

import imgviz
import IPython
import numpy as np
import pybullet as p
import pybullet_planning as pp

import reorientbot
from reorientbot.examples.reorientation import _env

import actionlib
from actionlib_msgs.msg import GoalStatus
import cv_bridge
from franka_msgs.msg import ErrorRecoveryAction
from franka_msgs.msg import ErrorRecoveryGoal
from franka_msgs.msg import FrankaState
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
import tf

from _message_subscriber import MessageSubscriber
from _panda import Panda
from _panda_ros_robot_interface import PandaROSRobotInterface


class BaseTaskInterface:
    def __init__(self):
        self._tf_listener = tf.listener.TransformListener(
            cache_time=rospy.Duration(60)
        )

        self._ri = PandaROSRobotInterface(robot=Panda())

        self._env = _env.Env(
            class_ids=None,
            real=True,
            robot_model="franka_panda/panda_drl",
            debug=False,
        )
        self._env.reset()

        self.real2robot()

        self._sub_points = MessageSubscriber(
            [
                ("/camera/color/camera_info", CameraInfo),
                ("/camera/color/image_rect_color", Image),
                ("/camera/aligned_depth_to_color/image_raw", Image),
            ],
            callback=self._sub_points_callback,
        )
        self._sub_points_density = 1 / 9
        self._sub_points_update_rate = 1
        self._sub_points_stamp = None
        self._sub_points_pybullet_id = None

        self._workspace_initialized = False

    def _sub_points_callback(self, info_msg, rgb_msg, depth_msg):
        if self._sub_points_stamp is not None and (
            info_msg.header.stamp - self._sub_points_stamp
        ) < rospy.Duration(1 / self._sub_points_update_rate):
            return

        K = np.array(info_msg.K).reshape(3, 3)
        bridge = cv_bridge.CvBridge()
        rgb = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        depth = bridge.imgmsg_to_cv2(depth_msg)
        assert depth.dtype == np.uint16
        depth = depth.astype(np.float32) / 1000
        depth[depth == 0] = np.nan

        try:
            camera_to_base = self.lookup_transform(
                "panda_link0",
                info_msg.header.frame_id,
                time=info_msg.header.stamp,
                timeout=rospy.Duration(1 / self._sub_points_update_rate),
            )
        except tf.ExtrapolationException:
            return

        pcd = reorientbot.geometry.pointcloud_from_depth(
            depth, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
        )
        pcd = reorientbot.geometry.transform_points(
            pcd, reorientbot.geometry.transformation_matrix(*camera_to_base)
        )

        height = int(round(rgb.shape[0] * np.sqrt(self._sub_points_density)))
        rgb = imgviz.resize(rgb, height=height)
        pcd = imgviz.resize(pcd, height=height)

        sub_points_pybullet_id = reorientbot.pybullet.draw_points(
            pcd, rgb, size=1
        )

        if self._sub_points_pybullet_id is not None:
            pp.remove_debug(self._sub_points_pybullet_id)
        self._sub_points_pybullet_id = sub_points_pybullet_id
        self._sub_points_stamp = info_msg.header.stamp

    @property
    def ri(self):
        return self._ri

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        if timeout is not None:
            self._tf_listener.waitForTransform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=time,
                timeout=timeout,
            )
        return self._tf_listener.lookupTransform(
            target_frame=target_frame, source_frame=source_frame, time=time
        )

    def recover_from_error(self):
        state_msg = rospy.wait_for_message(
            "/franka_state_controller/franka_states", FrankaState
        )
        if state_msg.robot_mode == FrankaState.ROBOT_MODE_MOVE:
            return True

        client = actionlib.SimpleActionClient(
            "/franka_control/error_recovery", ErrorRecoveryAction
        )
        client.wait_for_server()

        if client.get_state() == GoalStatus.SUCCEEDED:
            return True

        goal = ErrorRecoveryGoal()
        state = client.send_goal_and_wait(goal)
        succeeded = state == GoalStatus.SUCCEEDED

        if succeeded:
            rospy.loginfo("Recovered from error")
        else:
            rospy.logerr("Failed to recover from error")
        return succeeded

    def real2robot(self):
        self.ri.update_robot_state()
        self._env.pi.setj(self.ri.potentio_vector())
        for attachment in self._env.pi.attachments:
            attachment.assign()

    def visjs(self, js):
        for j in js:
            for _ in self._env.pi.movej(j):
                pp.step_simulation()
                time.sleep(1 / 240)

    def movejs(
        self, js, time_scale=None, wait=True, retry=False, wait_callback=None
    ):
        if not self.recover_from_error():
            return
        if time_scale is None:
            time_scale = 3
        js = np.asarray(js)

        self.real2robot()
        j_init = self._env.pi.getj()

        self.ri.angle_vector_sequence(
            js, time_scale=time_scale, max_pos_accel=1
        )
        if wait:
            success = self.wait_interpolation(callback=wait_callback)
            if success or not retry:
                return

            self.real2robot()
            j_curr = self._env.pi.getj()

            js = np.r_[[j_init], js]

            for i in range(len(js) - 1):
                dj1 = js[i + 1] - j_curr
                dj2 = js[i + 1] - js[i]
                dj1[abs(dj1) < 0.01] = 0
                dj2[abs(dj2) < 0.01] = 0
                if (np.sign(dj1) == np.sign(dj2)).all():
                    break
            else:
                return
            self.movejs(
                js[i + 1 :], time_scale=time_scale, wait=wait, retry=False
            )

    def wait_interpolation(self, callback=None):
        self._sub_points.subscribe()
        controller_actions = self.ri.controller_table[self.ri.controller_type]
        while True:
            states = [action.get_state() for action in controller_actions]
            if all(s >= GoalStatus.SUCCEEDED for s in states):
                break
            self.real2robot()
            if callback is not None:
                callback()
            rospy.sleep(0.01)
        self._sub_points.unsubscribe()
        if not all(s == GoalStatus.SUCCEEDED for s in states):
            rospy.logwarn("Some joint control requests have failed")
            return False
        return True

    def start_grasp(self):
        client = rospy.ServiceProxy("/set_suction", SetBool)
        client.call(data=True)

    def stop_grasp(self):
        client = rospy.ServiceProxy("/set_suction", SetBool)
        client.call(data=False)

    def reset_pose(self, *args, **kwargs):
        self.movejs([self._env.pi.homej], *args, **kwargs)

    def _solve_ik_for_look_at(self, eye, target, rotation_axis=True):
        c = reorientbot.geometry.Coordinate.from_matrix(
            reorientbot.geometry.look_at(eye, target)
        )
        if rotation_axis is True:
            for _ in range(4):
                c.rotate([0, 0, np.deg2rad(90)])
                if abs(c.euler[2] - np.deg2rad(-90)) < np.pi / 4:
                    break
        j = self._env.pi.solve_ik(
            c.pose,
            move_target=self._env.pi.robot_model.camera_link,
            n_init=20,
            thre=0.05,
            rthre=np.deg2rad(15),
            rotation_axis=rotation_axis,
            validate=True,
        )
        if j is None:
            rospy.logerr("j is not found")
            return
        return j

    def look_at(self, eye, target, rotation_axis=True, *args, **kwargs):
        j = self._solve_ik_for_look_at(eye, target, rotation_axis)
        self.movejs([j], *args, **kwargs)

    def init_workspace(self):
        if self._workspace_initialized:
            return

        # light
        p.configureDebugVisualizer(
            p.COV_ENABLE_SHADOWS, True, lightPosition=(100, -100, 0.5)
        )

        # table
        pp.set_texture(self._env.plane)

        # left wall
        obj = pp.create_box(w=3, l=0.01, h=1.05, color=(0.6, 0.6, 0.6, 1))
        pp.set_pose(
            obj,
            (
                (-0.0010000000000000002, 0.6925000000000028, 0.55),
                (0.0, 0.0, 0.0194987642109932, 0.9998098810245096),
            ),
        )
        self._env.bg_objects.append(obj)

        # back wall
        obj = pp.create_box(w=0.01, l=3, h=1.05, color=(0.7, 0.7, 0.7, 1))
        pp.set_pose(obj, ([-0.4, 0, 1.05 / 2], [0, 0, 0, 1]))
        self._env.bg_objects.append(obj)

        # ceiling
        obj = pp.create_box(w=3, l=3, h=0.5, color=(1, 1, 1, 1))
        pp.set_pose(obj, ([0, 0, 0.25 + 1.05], [0, 0, 0, 1]))
        self._env.bg_objects.append(obj)

        self._workspace_initialized = True

        # reorientbot.pybullet.annotate_pose(obj)


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-c", dest="cmd")
    args = parser.parse_args()

    rospy.init_node("base_task_interface")
    self = BaseTaskInterface()  # NOQA

    if args.cmd:
        exec(args.cmd)

    IPython.embed(header="base_task_interface")


if __name__ == "__main__":
    main()
