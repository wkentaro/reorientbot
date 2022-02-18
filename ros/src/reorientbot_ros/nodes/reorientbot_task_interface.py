#!/usr/bin/env python

import argparse
import itertools
import tempfile

import cv2
import imgviz
import IPython
import numpy as np
import path
import pybullet_planning as pp

import mercury
from mercury.examples.reorientation import _reorient
from mercury.examples.reorientation import _utils
from mercury.examples.reorientation.pickable_eval import (
    get_goal_oriented_reorient_poses,  # NOQA
)
from mercury.examples.reorientation.reorient_dynamic import (
    plan_dynamic_reorient,  # NOQA
)

import cv_bridge
from morefusion_ros.msg import ObjectClassArray
from morefusion_ros.msg import ObjectPoseArray
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_srvs.srv import Empty

from _message_subscriber import MessageSubscriber
from _tsdf_from_depth import tsdf_from_depth
from base_task_interface import BaseTaskInterface


class ReorientbotTaskInterface:

    pile_center = np.array([0.5, 0, 0])
    pile_center.setflags(write=False)

    def __init__(self, base: BaseTaskInterface):
        self.base = base

        self._sub_singleview = MessageSubscriber(
            [
                ("/camera/aligned_depth_to_color/image_raw", Image),
                (
                    "/camera/mask_rcnn_instance_segmentation/output/class",
                    ObjectClassArray,
                ),
                (
                    "/camera/mask_rcnn_instance_segmentation/output/label_ins",
                    Image,
                ),
                ("/singleview_3d_pose_estimation/output", ObjectPoseArray),
            ]
        )
        self._sub_multiview = MessageSubscriber(
            [("/object_mapping/output/poses", ObjectPoseArray)]
        )

        self._goals = None

    def run_singleview(self):
        self.base.init_workspace()

        if self._goals is None:
            self.init_task()

        for goal in self._goals:
            self._set_goal(goal)

            self.capture_pile_singleview()

            while True:
                result = self.pick_and_place()
                if "js_place" in result:
                    break

                self.pick_and_reorient()
                self.capture_target_singleview()

            # clear tsdf
            obj = self.base._env.bg_objects.pop(-1)
            pp.remove_body(obj)

    def capture_pile_singleview(self):
        self._look_at_pile()
        self._wait_for_message_singleview()
        self._process_message_singleview()

    def _look_at_pile(self, *args, **kwargs):
        self.base.look_at(
            eye=[0.5, 0, 0.7], target=[0.5, 0, 0], *args, **kwargs
        )

    def _start_passthrough_singleview(self):
        servers = ["/camera/color/image_rect_color_passthrough"]
        for server in servers:
            client = rospy.ServiceProxy(server + "/request", Empty)
            client.call()

    def _stop_passthrough_singleview(self):
        servers = ["/camera/color/image_rect_color_passthrough"]
        for server in servers:
            client = rospy.ServiceProxy(server + "/stop", Empty)
            client.call()

    def _wait_for_message_singleview(self):
        target_class_id = _utils.get_class_id(self._obj_goal)

        stamp = rospy.Time.now()
        self._sub_singleview.msgs = None
        self._sub_singleview.subscribe()
        self._start_passthrough_singleview()
        while True:
            rospy.sleep(0.1)
            if not self._sub_singleview.msgs:
                continue
            obj_poses_msg = self._sub_singleview.msgs[3]
            if obj_poses_msg.header.stamp < stamp:
                continue
            if target_class_id not in [
                pose.class_id for pose in obj_poses_msg.poses
            ]:
                continue
            break
        self._stop_passthrough_singleview()
        self._sub_singleview.unsubscribe()

    def _process_message_singleview(self, tsdf=True):
        cam_msg = rospy.wait_for_message(
            "/camera/color/camera_info", CameraInfo
        )
        (
            depth_msg,
            cls_msg,
            label_msg,
            obj_poses_msg,
        ) = self._sub_singleview.msgs

        K = np.array(cam_msg.K).reshape(3, 3)

        bridge = cv_bridge.CvBridge()
        depth = bridge.imgmsg_to_cv2(depth_msg)
        assert depth.dtype == np.uint16
        depth = depth.astype(np.float32) / 1000
        depth[depth == 0] = np.nan
        label = bridge.imgmsg_to_cv2(label_msg)

        camera_to_base = self.base.lookup_transform(
            "panda_link0",
            obj_poses_msg.header.frame_id,
            time=obj_poses_msg.header.stamp,
        )

        for obj_pose_msg in obj_poses_msg.poses:
            class_id = obj_pose_msg.class_id
            if class_id != _utils.get_class_id(self._obj_goal):
                continue

            pose = obj_pose_msg.pose
            obj_to_camera = (
                (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ),
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            )
            obj_to_base = pp.multiply(camera_to_base, obj_to_camera)

            visual_file = mercury.datasets.ycb.get_visual_file(class_id)
            collision_file = mercury.pybullet.get_collision_file(visual_file)
            if self.base._env.fg_object_id is None:
                obj = mercury.pybullet.create_mesh_body(
                    visual_file=visual_file,
                    collision_file=collision_file,
                    position=obj_to_base[0],
                    quaternion=obj_to_base[1],
                )
                self.base._env.fg_object_id = obj
                self.base._env.object_ids = [obj]
            else:
                pp.set_pose(self.base._env.fg_object_id, obj_to_base)
                assert self.base._env.fg_object_id in self.base._env.object_ids
            break
        else:
            raise RuntimeError("Target object is not found")

        if tsdf:
            target_instance_id = obj_pose_msg.instance_id

            mask = label == target_instance_id
            mask = (
                cv2.dilate(
                    imgviz.bool2ubyte(mask),
                    kernel=np.ones((8, 8)),
                    iterations=3,
                )
                == 255
            )
            depth_masked = depth.copy()
            depth_masked[mask] = np.nan
            tsdf = tsdf_from_depth(depth_masked, camera_to_base, K)
            with tempfile.TemporaryDirectory() as tmp_dir:
                visual_file = path.Path(tmp_dir) / "tsdf.obj"
                tsdf.export(visual_file)
                collision_file = mercury.pybullet.get_collision_file(
                    visual_file, resolution=10000
                )
                bg_structure = mercury.pybullet.create_mesh_body(
                    visual_file=visual_file,
                    collision_file=collision_file,
                    rgba_color=(0.5, 0.5, 0.5, 1),
                )
            self.base._env.bg_objects.append(bg_structure)

        self.base._env.update_obs()

    def capture_target_singleview(self):
        self._look_at_target()
        self._wait_for_message_singleview()
        self._process_message_singleview(tsdf=False)

    def _look_at_target(self):
        if self.base._env.fg_object_id is None:
            # default
            target = [0.2, -0.5, 0.1]
        else:
            target = pp.get_pose(self.base._env.fg_object_id)[0]
        self.base.look_at(
            eye=[target[0] - 0.1, target[1], target[2] + 0.5],
            target=target,
            rotation_axis="z",
            time_scale=4,
        )

    def pick_and_place(self, num_grasps=10):
        result = self._plan_place(num_grasps=num_grasps)
        if "js_place" not in result:
            rospy.logerr("Failed to plan placement")
            return result

        self.base.movejs(result["js_pre_grasp"], time_scale=5, retry=True)

        js = self.base._env.pi.get_cartesian_path(j=result["j_grasp"])

        if _utils.get_class_id(self.base._env.fg_object_id) == 5:
            # likely to move
            self.base.movejs(js[:-2], time_scale=10)
            self.base.start_grasp()
            self.base.movejs(js[-2:], time_scale=10)
        else:
            self.base.movejs(js, time_scale=10)
            self.base.start_grasp()

        rospy.sleep(1)
        self.base._env.pi.attachments = result["attachments"]

        self.base.movejs(result["js_pre_place"], time_scale=5, retry=True)

        self.base.movejs(result["js_place"], time_scale=10)

        self.base.stop_grasp()
        rospy.sleep(9)
        self.base._env.pi.attachments = []

        self.base.movejs(result["js_post_place"], time_scale=10, retry=True)

        js = self.base._env.pi.planj(
            self.base._env.pi.homej,
            obstacles=self.base._env.bg_objects + self.base._env.object_ids,
        )
        if js is None:
            self.base.reset_pose(time_scale=8, retry=True)
        else:
            self.base.movejs(js, time_scale=5, retry=True)

        return result

    def _plan_place(self, num_grasps):
        pcd_in_obj, normals_in_obj = _reorient.get_query_ocs(self.base._env)
        dist_from_centroid = np.linalg.norm(pcd_in_obj, axis=1)

        indices = np.arange(pcd_in_obj.shape[0])
        p = dist_from_centroid.max() - dist_from_centroid

        keep = dist_from_centroid < np.median(dist_from_centroid)
        indices = indices[keep]
        p = p[keep]
        if _utils.get_class_id(self.base._env.fg_object_id) in [5, 11]:
            indices = np.r_[
                np.random.choice(indices, num_grasps, p=p / p.sum()),
            ]
        else:
            indices = np.r_[
                np.random.choice(indices, num_grasps // 2, p=p / p.sum()),
                np.random.permutation(pcd_in_obj.shape[0])[: num_grasps // 2],
            ]

        pcd_in_obj = pcd_in_obj[indices]
        normals_in_obj = normals_in_obj[indices]
        quaternion_in_obj = mercury.geometry.quaternion_from_vec2vec(
            [0, 0, -1], normals_in_obj
        )
        grasp_poses = np.hstack([pcd_in_obj, quaternion_in_obj])  # in obj

        if 0:
            for grasp_pose in grasp_poses:
                pp.draw_pose(
                    np.hsplit(grasp_pose, [3]),
                    parent=self._obj_goal,
                    width=2,
                    length=0.05,
                )

        result = _reorient.plan_place(self.base._env, grasp_poses)

        return result

    def pick_and_reorient(self, heuristic=False):
        result = self._plan_reorient(heuristic=heuristic)
        if "js_place" not in result:
            rospy.logerr("Failed to plan reorientation")
            return result

        self.base.movejs(result["js_pre_grasp"], time_scale=5, retry=True)

        js = self.base._env.pi.get_cartesian_path(j=result["j_grasp"])

        if _utils.get_class_id(self.base._env.fg_object_id) == 5:
            # likely to move
            self.base.movejs(js[:-2], time_scale=10)
            self.base.start_grasp()
            self.base.movejs(js[-2:], time_scale=10)
        else:
            self.base.movejs(js, time_scale=10)
            self.base.start_grasp()

        rospy.sleep(1)
        self.base._env.pi.attachments = result["attachments"]

        js = result["js_place"]
        self.base.movejs(js, time_scale=5, retry=True)

        with pp.WorldSaver():
            self.base._env.pi.setj(js[-1])
            c = mercury.geometry.Coordinate(
                *self.base._env.pi.get_pose("tipLink")
            )
            js = []
            for i in range(3):
                c.translate([0, 0, -0.01], wrt="world")
                j = self.base._env.pi.solve_ik(c.pose, rotation_axis=None)
                if j is not None:
                    js.append(j)
        self.base.movejs(js, time_scale=10, wait=False)

        self.base.stop_grasp()

        rospy.sleep(6)
        self.base._env.pi.attachments = []

        js = result["js_post_place"]
        self.base.movejs(js, time_scale=5)

        self.base.movejs([self.base._env.pi.homej], time_scale=3, retry=True)

        return result

    def _plan_reorient(self, heuristic=False):
        if heuristic:
            grasp_poses = _reorient.get_grasp_poses(self.base._env)
            grasp_poses = list(itertools.islice(grasp_poses, 12))
            reorient_poses = _reorient.get_static_reorient_poses(
                self.base._env
            )

            result = {}
            for grasp_pose, reorient_pose in itertools.product(
                grasp_poses, reorient_poses
            ):
                result = _reorient.plan_reorient(
                    self.base._env, grasp_pose, reorient_pose
                )
                if "js_place" in result:
                    break
            else:
                rospy.logerr("No solution found")
        else:
            (
                reorient_poses,
                pickable,
                target_grasp_poses,
            ) = get_goal_oriented_reorient_poses(self.base._env)

            grasp_poses = _reorient.get_grasp_poses(self.base._env)  # in world
            grasp_poses = list(itertools.islice(grasp_poses, 25))

            for threshold in np.linspace(0.99, 0.5):
                indices = np.where(pickable > threshold)[0]
                if indices.size > 100:
                    break
            indices = np.random.choice(
                indices, min(indices.size, 1000), replace=False
            )
            reorient_poses = reorient_poses[indices]
            pickable = pickable[indices]

            result = plan_dynamic_reorient(
                self.base._env,
                grasp_poses,
                reorient_poses,
                pickable,
            )
        return result

    def _set_goal(self, goal):
        self.base._env.fg_object_id = None
        self.base._env.PLACE_POSE = goal["place_pose"]
        self.base._env.LAST_PRE_PLACE_POSE = goal["last_pre_place_pose"]
        self.base._env.PRE_PLACE_POSE = goal["pre_place_pose"]
        self._obj_goal = goal["obj_goal"]

    def run_multiview(self):
        self.base.init_workspace()

        if self._goals is None:
            self.init_task()

        self.base.reset_pose(time_scale=10)

        self.capture_pile_multiview()

        for goal in self._goals:
            self._set_goal(goal)

            target_class_id = _utils.get_class_id(self._obj_goal)

            if target_class_id not in [
                _utils.get_class_id(o) for o in self.base._env.object_ids
            ]:
                rospy.logerr(f"Target object {target_class_id} is not found")
                continue

            for object_id in self.base._env.object_ids:
                if _utils.get_class_id(object_id) != target_class_id:
                    continue

                self.base._env.fg_object_id = object_id

                target = np.array(pp.get_pose(self.base._env.fg_object_id)[0])
                j = self.base._solve_ik_for_look_at(
                    eye=target + [0, 0, 0.5],
                    target=target,
                    rotation_axis="z",
                )
                j_bak = self.base._env.pi.getj()
                self.base._env.pi.setj(j)
                self.base._env.update_obs()
                self.base._env.pi.setj(j_bak)

                if "js_place" in self.pick_and_place():
                    break
            else:
                for object_id in self.base._env.object_ids:
                    if _utils.get_class_id(object_id) != target_class_id:
                        continue

                    self.base._env.fg_object_id = object_id

                    target = np.array(
                        pp.get_pose(self.base._env.fg_object_id)[0]
                    )
                    j = self.base._solve_ik_for_look_at(
                        eye=target + [0, 0, 0.5],
                        target=target,
                        rotation_axis="z",
                    )
                    j_bak = self.base._env.pi.getj()
                    self.base._env.pi.setj(j)
                    self.base._env.update_obs()
                    self.base._env.pi.setj(j_bak)

                    if "js_place" in self.pick_and_reorient():
                        break

                self.capture_target_singleview()
                self.pick_and_place(num_grasps=20)

            self.base._env.object_ids.remove(self.base._env.fg_object_id)
            self.base._env.fg_object_id = None

    def capture_pile_multiview(self):
        self._reset_multiview()

        dxdy = [
            # center-bottom
            (-0.2, +0.0),
            # left-bottom -> left-top
            (-0.2, +0.3),
            (+0.0, +0.3),
            (+0.1, +0.3),
            # center-top -> center-bottom
            (+0.1, +0.0),
            (+0.0, +0.0),
            (-0.2, +0.0),
            # # right-bottom -> right-top
            # (-0.2, -0.2),
            # (+0.0, -0.2),
        ]
        with pp.WorldSaver():
            self.base._env.pi.setj(self.base._env.pi.homej)

            js = []
            for i, (dx, dy) in enumerate(dxdy):
                rotation_axis = True if i == 0 else "z"
                j = self.base._solve_ik_for_look_at(
                    eye=self.pile_center + [dx, dy, 0.7],
                    target=self.pile_center + [dx / 2, dy / 2, 0],
                    rotation_axis=rotation_axis,
                )
                js.append(j)
                self.base._env.pi.setj(j)

        instance_id_to_object_id = {}

        def wait_callback():
            if self._sub_multiview.msgs is None:
                return
            (obj_poses_msg,) = self._sub_multiview.msgs
            for obj_pose_msg in obj_poses_msg.poses:
                instance_id = obj_pose_msg.instance_id
                if instance_id in instance_id_to_object_id:
                    # already spawned
                    continue

                class_id = obj_pose_msg.class_id
                pose_msg = obj_pose_msg.pose
                position = [getattr(pose_msg.position, key) for key in "xyz"]
                quaternion = [
                    getattr(pose_msg.orientation, key) for key in "xyzw"
                ]
                obj = mercury.pybullet.create_mesh_body(
                    visual_file=mercury.datasets.ycb.get_visual_file(
                        class_id=class_id
                    ),
                    collision_file=True,
                    position=position,
                    quaternion=quaternion,
                )
                self.base._env.object_ids.append(obj)
                instance_id_to_object_id[instance_id] = obj

        self.base._env.object_ids = []
        self._sub_multiview.msgs = None
        self._sub_multiview.subscribe()
        self._start_passthrough_multiview()
        self.base.movejs([js[0]], wait_callback=wait_callback)
        stamp = rospy.Time.now()
        rospy.sleep(1)
        while self._sub_multiview.msgs is None:
            rospy.sleep(0.1)
        while self._sub_multiview.msgs[0].header.stamp < stamp:
            rospy.sleep(0.1)
        self.base.movejs(
            js, time_scale=12, wait_callback=wait_callback, retry=True
        )
        self._stop_passthrough_multiview()
        self._sub_multiview.unsubscribe()

    def _reset_multiview(self):
        self._stop_passthrough_multiview()
        servers = [
            "/camera/octomap_server",
            "/object_mapping",
        ]
        for server in servers:
            client = rospy.ServiceProxy(server + "/reset", Empty)
            client.call()

    def _start_passthrough_multiview(self):
        servers = [
            "/camera/color/image_rect_color_passthrough",
            "/camera/depth_registered/points_passthrough",
        ]
        for server in servers:
            client = rospy.ServiceProxy(server + "/request", Empty)
            client.call()

    def _stop_passthrough_multiview(self):
        servers = [
            "/camera/color/image_rect_color_passthrough",
            "/camera/depth_registered/points_passthrough",
        ]
        for server in servers:
            client = rospy.ServiceProxy(server + "/stop", Empty)
            client.call()

    def init_task(self):
        shelf1 = _utils.create_shelf(X=0.29, Y=0.41, Z=0.285, N=2)
        mercury.pybullet.set_pose(
            shelf1,
            (
                (
                    0.30150000000000315,
                    0.5360000000000005,
                    0.009900000000000003,
                ),
                (0.0, 0.0, -0.7071067811865478, 0.7071067811865472),
            ),
        )

        shelf2 = _utils.create_shelf(X=0.29, Y=0.41, Z=0.285, N=2)
        mercury.pybullet.set_pose(
            shelf2,
            (
                (
                    0.7526728391044942,
                    0.31192276483837894,
                    0.011299999999999994,
                ),
                (0.0, 0.0, 0.9422915766421891, -0.33479334609453854),
            ),
        )

        color = (0.7, 0.7, 0.7, 1)
        create = None  # [0, 1, 2]

        box1 = mercury.pybullet.create_bin(
            X=0.3, Y=0.3, Z=0.11, color=color, create=create
        )
        c = mercury.geometry.Coordinate()
        c.rotate([np.deg2rad(9), 0, 0])
        c.rotate([0, 0, np.deg2rad(-110)], wrt="world")
        c.translate([0.85, -0.15, 0.09], wrt="world")
        pp.set_pose(box1, c.pose)

        box2 = mercury.pybullet.create_bin(
            X=0.3, Y=0.3, Z=0.11, color=color, create=create
        )
        box1_to_world = pp.get_pose(box1)
        c = mercury.geometry.Coordinate()
        c.translate([0.31, 0, 0])
        box2_to_box1 = c.pose
        box2_to_world = pp.multiply(box1_to_world, box2_to_box1)
        pp.set_pose(box2, box2_to_world)

        self.base._env.bg_objects.append(shelf1)
        self.base._env.bg_objects.append(shelf2)
        self.base._env.bg_objects.append(box1)
        self.base._env.bg_objects.append(box2)

        # -----------------------------------------------------------------------------

        goals = []

        # 1st
        class_id = 2
        obj = mercury.pybullet.create_mesh_body(
            visual_file=mercury.datasets.ycb.get_visual_file(class_id),
            rgba_color=(1, 1, 1, 0.5),
            mesh_scale=(0.99, 0.99, 0.99),
        )
        c = mercury.geometry.Coordinate(
            quaternion=_utils.get_canonical_quaternion(class_id)
        )
        c.rotate([0, 0, np.deg2rad(90)])
        c.translate([0.06, 0.16, 0.43], wrt="world")
        obj_to_shelf2 = c.pose
        shelf2_to_world = pp.get_pose(shelf2)
        obj_to_world = pp.multiply(shelf2_to_world, obj_to_shelf2)
        pp.set_pose(obj, obj_to_world)
        place_pose = obj_to_world
        c.translate([0, -0.05, 0.02], wrt="world")
        last_pre_place_pose = pp.multiply(shelf2_to_world, c.pose)
        c.translate([0.25, 0, 0.05], wrt="world")
        pre_place_pose = pp.multiply(shelf2_to_world, c.pose)
        goals.append(
            dict(
                place_pose=place_pose,
                last_pre_place_pose=last_pre_place_pose,
                pre_place_pose=pre_place_pose,
                obj_goal=obj,
            ),
        )

        # 2nd
        class_id = 2
        obj = mercury.pybullet.create_mesh_body(
            visual_file=mercury.datasets.ycb.get_visual_file(class_id),
            rgba_color=(1, 1, 1, 0.5),
            mesh_scale=(0.99, 0.99, 0.99),
        )
        c = mercury.geometry.Coordinate(
            quaternion=_utils.get_canonical_quaternion(class_id)
        )
        c.rotate([0, 0, np.deg2rad(90)])
        c.translate([0.06, 0.10, 0.43], wrt="world")
        obj_to_shelf2 = c.pose
        shelf2_to_world = pp.get_pose(shelf2)
        obj_to_world = pp.multiply(shelf2_to_world, obj_to_shelf2)
        pp.set_pose(obj, obj_to_world)
        place_pose = obj_to_world
        c.translate([0, -0.05, 0.02], wrt="world")
        last_pre_place_pose = pp.multiply(shelf2_to_world, c.pose)
        c.translate([0.25, 0, 0.05], wrt="world")
        pre_place_pose = pp.multiply(shelf2_to_world, c.pose)
        goals.append(
            dict(
                place_pose=place_pose,
                last_pre_place_pose=last_pre_place_pose,
                pre_place_pose=pre_place_pose,
                obj_goal=obj,
            ),
        )

        # 3rd
        class_id = 2
        obj = mercury.pybullet.create_mesh_body(
            visual_file=mercury.datasets.ycb.get_visual_file(class_id),
            rgba_color=(1, 1, 1, 0.5),
            mesh_scale=(0.99, 0.99, 0.99),
        )
        c = mercury.geometry.Coordinate(
            quaternion=_utils.get_canonical_quaternion(class_id)
        )
        c.translate([0.105, -0.12, 0.43], wrt="world")
        obj_to_shelf2 = c.pose
        shelf2_to_world = pp.get_pose(shelf2)
        obj_to_world = pp.multiply(shelf2_to_world, obj_to_shelf2)
        pp.set_pose(obj, obj_to_world)
        place_pose = obj_to_world
        c.translate([0, 0.02, 0.02], wrt="world")
        last_pre_place_pose = pp.multiply(shelf2_to_world, c.pose)
        c.translate([0.3, 0, 0.05], wrt="world")
        pre_place_pose = pp.multiply(shelf2_to_world, c.pose)
        goals.append(
            dict(
                place_pose=place_pose,
                last_pre_place_pose=last_pre_place_pose,
                pre_place_pose=pre_place_pose,
                obj_goal=obj,
            ),
        )

        # 4th
        class_id = 5
        obj = mercury.pybullet.create_mesh_body(
            visual_file=mercury.datasets.ycb.get_visual_file(class_id),
            rgba_color=(1, 1, 1, 0.5),
            mesh_scale=(0.99, 0.99, 0.99),
        )
        c = mercury.geometry.Coordinate(
            quaternion=_utils.get_canonical_quaternion(class_id)
        )
        c.translate([0.10, 0.08, 0.39], wrt="world")
        obj_to_shelf1 = c.pose
        shelf1_to_world = pp.get_pose(shelf1)
        obj_to_world = pp.multiply(shelf1_to_world, obj_to_shelf1)
        pp.set_pose(obj, obj_to_world)
        place_pose = obj_to_world
        c.translate([0, 0.0, 0.05], wrt="world")
        last_pre_place_pose = pp.multiply(shelf1_to_world, c.pose)
        c.translate([0.3, 0, 0.05], wrt="world")
        pre_place_pose = pp.multiply(shelf1_to_world, c.pose)
        goals.append(
            dict(
                place_pose=place_pose,
                last_pre_place_pose=last_pre_place_pose,
                pre_place_pose=pre_place_pose,
                obj_goal=obj,
            ),
        )

        # 5th
        class_id = 11
        obj = mercury.pybullet.create_mesh_body(
            visual_file=mercury.datasets.ycb.get_visual_file(class_id),
            rgba_color=(1, 1, 1, 0.5),
            mesh_scale=(0.99, 0.99, 0.99),
        )
        c = mercury.geometry.Coordinate(
            quaternion=_utils.get_canonical_quaternion(class_id)
        )
        c.rotate([0, 0, np.deg2rad(-70)])
        c.rotate([np.deg2rad(-90), 0, 0], wrt="world")
        c.translate([0.01, 0.03, 0.02], wrt="world")
        obj_to_box2 = c.pose
        box2_to_world = pp.get_pose(box2)
        obj_to_world = pp.multiply(box2_to_world, obj_to_box2)
        pp.set_pose(obj, obj_to_world)
        place_pose = obj_to_world
        c.translate([0, 0.0, 0.2], wrt="world")
        pre_place_pose = pp.multiply(box2_to_world, c.pose)
        goals.append(
            dict(
                place_pose=place_pose,
                last_pre_place_pose=None,
                pre_place_pose=pre_place_pose,
                obj_goal=obj,
            ),
        )

        # 6th
        class_id = 3
        obj = mercury.pybullet.create_mesh_body(
            visual_file=mercury.datasets.ycb.get_visual_file(class_id),
            rgba_color=(1, 1, 1, 0.5),
            mesh_scale=(0.99, 0.99, 0.99),
        )
        c = mercury.geometry.Coordinate(
            quaternion=_utils.get_canonical_quaternion(class_id)
        )
        c.rotate([0, 0, np.deg2rad(-90)])
        c.rotate([np.deg2rad(-90), 0, 0], wrt="world")
        c.translate([-0.01, 0.02, -0.02], wrt="world")
        obj_to_box1 = c.pose
        box1_to_world = pp.get_pose(box1)
        obj_to_world = pp.multiply(box1_to_world, obj_to_box1)
        pp.set_pose(obj, obj_to_world)
        place_pose = obj_to_world
        c.translate([0.0, 0.0, 0.2], wrt="world")
        pre_place_pose = pp.multiply(box1_to_world, c.pose)
        goals.append(
            dict(
                place_pose=place_pose,
                last_pre_place_pose=None,
                pre_place_pose=pre_place_pose,
                obj_goal=obj,
            ),
        )

        self._goals = goals


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-c", dest="cmd")
    args = parser.parse_args()

    rospy.init_node("reorientbot_task_interface")
    base = BaseTaskInterface()
    self = ReorientbotTaskInterface(base)  # NOQA

    if args.cmd:
        exec(args.cmd)

    IPython.embed()


if __name__ == "__main__":
    main()
