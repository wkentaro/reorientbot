import contextlib
import itertools

from loguru import logger
import numpy as np
import path
import pybullet as p
import pybullet_planning

from .. import geometry
from .. import utils
from . import utils as pybullet_utils
from .ompl_planning import PbPlanner
from .suction_gripper import SuctionGripper

import skrobot


here = path.Path(__file__).abspath().parent


class PandaRobotInterface:
    def __init__(
        self,
        pose=None,
        suction_max_force=10,
        suction_surface_threshold=np.deg2rad(10),
        suction_surface_alignment=True,
        planner="RRTConnect",
        robot_model="franka_panda/panda_suction",
    ):
        self.pose = pose

        urdf_file = here / f"data/{robot_model}.urdf"
        self.robot_model = skrobot.models.urdf.RobotModelFromURDF(
            urdf_file=urdf_file
        )
        self.robot = pybullet_planning.load_pybullet(
            urdf_file, fixed_base=True
        )
        self.ee = pybullet_planning.link_from_name(self.robot, "tipLink")

        self.gripper = SuctionGripper(
            self.robot,
            self.ee,
            max_force=suction_max_force,
            surface_threshold=suction_surface_threshold,
            surface_alignment=suction_surface_alignment,
        )

        self.attachments = []

        if self.pose is not None:
            self.robot_model.translate(pose[0])
            self.robot_model.orient_with_matrix(
                geometry.quaternion_matrix(pose[1])[:3, :3]
            )

            pybullet_planning.set_pose(self.robot, self.pose)

        # Get revolute joint indices of robot (skip fixed joints).
        n_joints = p.getNumJoints(self.robot)
        joints = [p.getJointInfo(self.robot, i) for i in range(n_joints)]
        self.joints = [j[0] for j in joints if j[2] == p.JOINT_REVOLUTE]

        self.homej = [0, -np.pi / 4, 0, -np.pi / 2, 0, np.pi / 4, np.pi / 4]
        for joint, joint_angle in zip(self.joints, self.homej):
            p.resetJointState(self.robot, joint, joint_angle)
        self.update_robot_model()

        self.planner = planner

        lower, upper = self.get_bounds()
        for joint, min_angle, max_angle in zip(self.joints, lower, upper):
            joint_name = pybullet_planning.get_joint_name(
                self.robot, joint
            ).decode()
            getattr(self.robot_model, joint_name).min_angle = min_angle
            getattr(self.robot_model, joint_name).max_angle = max_angle

    def get_bounds(self):
        lower_bounds = []
        upper_bounds = []
        for joint in self.joints:
            lower, upper = p.getJointInfo(self.robot, joint)[8:10]
            center = (upper + lower) / 2
            width = upper - lower
            width = width * 0.96
            upper = center + width / 2
            lower = center - width / 2
            lower_bounds.append(lower)
            upper_bounds.append(upper)
        lower_bounds = np.array(lower_bounds)
        upper_bounds = np.array(upper_bounds)
        return lower_bounds, upper_bounds

    def step_simulation(self):
        self.gripper.step_simulation()

    def update_robot_model(self, j=None):
        if j is None:
            j = self.getj()
        for joint, joint_angle in zip(self.joints, j):
            joint_name = pybullet_planning.get_joint_name(
                self.robot, joint
            ).decode()
            getattr(self.robot_model, joint_name).joint_angle(joint_angle)

    def setj(self, joint_positions):
        for joint, joint_position in zip(self.joints, joint_positions):
            p.resetJointState(self.robot, joint, joint_position)
        for attachment in self.attachments:
            attachment.assign()

    def getj(self):
        joint_positions = []
        for joint in self.joints:
            joint_positions.append(p.getJointState(self.robot, joint)[0])
        return joint_positions

    def movej(self, targj, speed=0.01, timeout=5, raise_on_timeout=False):
        assert len(targj) == len(self.joints)
        for i in itertools.count():
            currj = [p.getJointState(self.robot, i)[0] for i in self.joints]
            currj = np.array(currj)
            diffj = targj - currj
            if all(np.abs(diffj) < 1e-2):
                return

            # Move with constant velocity
            norm = np.linalg.norm(diffj)
            v = diffj / norm if norm > 0 else 0
            stepj = currj + v * speed
            gains = np.ones(len(self.joints))
            p.setJointMotorControlArray(
                bodyIndex=self.robot,
                jointIndices=self.joints,
                controlMode=p.POSITION_CONTROL,
                targetPositions=stepj,
                positionGains=gains,
            )
            yield i

            if i >= (timeout / pybullet_planning.get_time_step()):
                if raise_on_timeout:
                    raise RuntimeError("timeout in joint motor control")
                else:
                    logger.error("timeout in joint motor control")
                    return

    def solve_ik(
        self,
        pose,
        move_target=None,
        n_init=1,
        random_state=None,
        validate=False,
        obstacles=None,
        **kwargs,
    ):
        if move_target is None:
            move_target = self.robot_model.tipLink
        if random_state is None:
            random_state = np.random.RandomState()

        def sample_fn():
            lower, upper = self.get_bounds()
            extents = upper - lower
            scale = random_state.uniform(size=len(lower))
            return lower + scale * extents

        self.update_robot_model()
        c = geometry.Coordinate(*pose)
        for _ in range(n_init):
            result = self.robot_model.inverse_kinematics(
                c.skrobot_coords,
                move_target=move_target,
                **kwargs,
            )
            if result is not False:
                if not validate:
                    break
                if self.validatej(result, obstacles=obstacles):
                    break
            self.update_robot_model(sample_fn())
        else:
            # logger.warning("Failed to solve IK")
            return
        j = []
        for joint in self.joints:
            joint_name = pybullet_planning.get_joint_name(
                self.robot, joint
            ).decode()
            j.append(getattr(self.robot_model, joint_name).joint_angle())
        return j

    # def _solve_ik_pybullet(self, pose):
    #     n_joints = p.getNumJoints(self.robot)
    #     lower_limits = []
    #     upper_limits = []
    #     for i in range(n_joints):
    #         joint_info = p.getJointInfo(self.robot, i)
    #         lower_limits.append(joint_info[8])
    #         upper_limits.append(joint_info[9])
    #     joint_positions = p.calculateInverseKinematics(
    #         bodyUniqueId=self.robot,
    #         endEffectorLinkIndex=self.ee,
    #         targetPosition=pose[0],
    #         targetOrientation=pose[1],
    #         lowerLimits=lower_limits,
    #         upperLimits=upper_limits,
    #         restPoses=self.homej,
    #         maxNumIterations=1000,
    #         residualThreshold=1e-5,
    #     )
    #     return joint_positions
    #
    # def _solve_ik_pybullet_planning(self, pose):
    #     with pybullet_planning.LockRenderer():
    #         with pybullet_planning.WorldSaver():
    #             joint_positions = pybullet_planning.inverse_kinematics(
    #                 self.robot,
    #                 self.ee,
    #                 pose,
    #             )
    #     return joint_positions

    @contextlib.contextmanager
    def enabling_attachments(self):
        robot_model = self.robot_model
        try:
            self.robot_model = self.get_skrobot(attachments=self.attachments)
            yield
        finally:
            self.robot_model = robot_model

    def get_skrobot(self, attachments=None):
        attachments = attachments or []

        self.update_robot_model()

        link_list = self.robot_model.link_list.copy()
        joint_list = self.robot_model.joint_list.copy()
        for i, attachment in enumerate(attachments):
            position, quaternion = attachment.grasp_pose
            link = skrobot.model.Link(
                parent=self.robot_model.tipLink,
                pos=position,
                rot=geometry.quaternion_matrix(quaternion)[:3, :3],
                name=f"attachment_link{i}",
            )
            joint = skrobot.model.FixedJoint(
                child_link=link,
                parent_link=self.robot_model.tipLink,
                name=f"attachment_joint{i}",
            )
            link.joint = joint
            link_list.append(link)
            joint_list.append(joint)
        return skrobot.model.RobotModel(
            link_list=link_list,
            joint_list=joint_list,
            # root_link=self.robot_model.root_link,
        )

    def validatej(self, j, obstacles=None, min_distances=None):
        planner = PbPlanner(
            self,
            obstacles=obstacles,
            min_distances=min_distances,
            planner=self.planner,
        )
        return planner.validityChecker.isValid(j)

    def planj(
        self,
        j,
        obstacles=None,
        min_distances=None,
        min_distances_start_goal=None,
        planner_range=0,
    ):
        if self.planner == "Naive":
            return [j]

        planner = PbPlanner(
            self,
            obstacles=obstacles,
            min_distances=min_distances,
            min_distances_start_goal=min_distances_start_goal,
            planner=self.planner,
            planner_range=planner_range,
        )

        planner.validityChecker.start = self.getj()
        planner.validityChecker.goal = j

        if not planner.validityChecker.isValid(self.getj()):
            logger.warning("Start state is invalid")
            return

        if not planner.validityChecker.isValid(j):
            logger.warning("Goal state is invalid")
            return

        result = planner.plan(self.getj(), j)

        if result is None:
            logger.warning("No solution found")
            return

        ndof = len(self.joints)
        state_count = result.getStateCount()
        path = np.zeros((state_count, ndof), dtype=float)
        for i_state in range(state_count):
            state = result.getState(i_state)
            path_i = np.zeros((ndof,), dtype=float)
            for i_dof in range(ndof):
                path_i[i_dof] = state[i_dof]
            path[i_state] = path_i

        if not np.allclose(j, path[-1]):
            # the goal is not reached
            return

        return path

    def grasp(self, min_dz=None, max_dz=None, rotation_axis="z", speed=0.01):
        c = geometry.Coordinate(
            *pybullet_planning.get_link_pose(self.robot, self.ee)
        )
        dz_done = 0
        while True:
            c.translate([0, 0, 0.001])
            dz_done += 0.001
            j = self.solve_ik(c.pose, rotation_axis=rotation_axis)
            if j is None:
                raise RuntimeError("IK failed")
            for i in self.movej(j, speed=speed):
                yield i
                if min_dz is not None and dz_done < min_dz:
                    continue
                if self.gripper.detect_contact():
                    break
            if min_dz is not None and dz_done < min_dz:
                continue
            if self.gripper.detect_contact():
                break
            if max_dz is not None and dz_done >= max_dz:
                break
        self.gripper.activate()

    def ungrasp(self):
        self.gripper.release()
        # if hasattr(self, "virtual_grasped_object"):
        #     p.removeBody(self.virtual_grasped_object)
        #     del self.virtual_grasped_object
        self.attachments = []

    def add_link(self, name, pose, parent=None):
        if parent is None:
            parent = self.ee
        parent_name = pybullet_planning.get_link_name(self.robot, parent)

        link_list = self.robot_model.link_list.copy()
        joint_list = self.robot_model.joint_list.copy()
        parent_link = getattr(self.robot_model, parent_name)
        link = skrobot.model.Link(
            parent=parent_link,
            pos=pose[0],
            rot=geometry.quaternion_matrix(pose[1])[:3, :3],
            name=name,
        )
        joint = skrobot.model.FixedJoint(
            child_link=link,
            parent_link=parent_link,
            name=f"{parent_name}_to_{name}_joint",
        )
        link.joint = joint
        link_list.append(link)
        joint_list.append(joint)
        self.robot_model = skrobot.model.RobotModel(
            link_list=link_list,
            joint_list=joint_list,
            # root_link=self.robot_model.root_link,
        )

    def get_pose(self, name):
        self.update_robot_model()
        T_a_to_world = getattr(self.robot_model, name).worldcoords().T()
        a_to_world = geometry.Coordinate.from_matrix(T_a_to_world).pose
        return a_to_world

    def add_camera(
        self,
        pose,
        fovy=np.deg2rad(42),
        height=480,
        width=640,
        parent=None,
    ):
        if parent is None:
            parent = self.ee
        self.add_link("camera_link", pose=pose, parent=parent)

        # pybullet_planning.draw_pose(
        #     pose, parent=self.robot, parent_link=parent
        # )
        pybullet_utils.draw_camera(
            fovy=fovy,
            height=height,
            width=width,
            pose=pose,
            parent=self.robot,
            parent_link=parent,
        )

        self.camera = dict(fovy=fovy, height=height, width=width)

    def get_camera_image(self):
        if not hasattr(self.robot_model, "camera_link"):
            raise ValueError

        self.update_robot_model()
        return pybullet_utils.get_camera_image(
            T_cam2world=self.robot_model.camera_link.worldcoords().T(),
            fovy=self.camera["fovy"],
            height=self.camera["height"],
            width=self.camera["width"],
        )

    def get_opengl_intrinsic_matrix(self):
        return geometry.opengl_intrinsic_matrix(
            fovy=self.camera["fovy"],
            height=self.camera["height"],
            width=self.camera["width"],
        )

    def random_grasp(
        self,
        depth,
        segm,
        mask,
        bg_object_ids,
        object_ids,
        max_angle=np.deg2rad(45),
        num_trial=10,
        random_state=None,
        noise=True,
    ):
        if random_state is None:
            random_state = np.random.RandomState()

        # This should be called after moving camera to observe the scene.
        K = self.get_opengl_intrinsic_matrix()

        if mask.sum() == 0:
            logger.warning("mask is empty")
            return

        pcd_in_camera = geometry.pointcloud_from_depth(
            depth, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
        )

        camera_to_world = self.get_pose("camera_link")
        ee_to_world = self.get_pose("tipLink")
        camera_to_ee = pybullet_planning.multiply(
            pybullet_planning.invert(ee_to_world), camera_to_world
        )
        pcd_in_ee = geometry.transform_points(
            pcd_in_camera,
            geometry.transformation_matrix(*camera_to_ee),
        )

        normals = geometry.normals_from_pointcloud(pcd_in_ee)

        segm = segm.reshape(-1)
        mask = mask.reshape(-1)
        pcd_in_ee = pcd_in_ee.reshape(-1, 3)
        normals = normals.reshape(-1, 3)

        indices = np.where(mask)[0]
        random_state.shuffle(indices)

        j_init = self.getj()

        path1 = path2 = None
        for index in indices[:num_trial]:
            object_id = segm[index]
            position = pcd_in_ee[index]
            quaternion = geometry.quaternion_from_vec2vec(
                [0, 0, 1], normals[index]
            )
            T_ee_to_ee_af_in_ee = geometry.transformation_matrix(
                position, quaternion
            )

            T_ee_to_world = geometry.transformation_matrix(
                *pybullet_utils.get_pose(self.robot, self.ee)
            )
            T_ee_to_ee = np.eye(4)
            T_ee_af_to_ee = T_ee_to_ee_af_in_ee @ T_ee_to_ee
            T_ee_af_to_world = T_ee_to_world @ T_ee_af_to_ee

            c = geometry.Coordinate(
                *geometry.pose_from_matrix(T_ee_af_to_world)
            )
            c.translate([0, 0, -0.1])

            vec = geometry.transform_points([[0, 0, 0], [0, 0, 1]], c.matrix)
            if 0:
                pybullet_planning.add_line(vec[0], vec[1], width=3)
            v0 = [0, 0, -1]
            v1 = vec[1] - vec[0]
            v1 /= np.linalg.norm(v1)
            angle = geometry.angle_between_vectors(v0, v1)
            if angle > max_angle:
                # logger.warning(f"angle > {np.rad2deg(max_angle)} deg")
                continue

            j = self.solve_ik(
                c.pose, rotation_axis="z", random_state=random_state
            )
            if j is None:
                # logger.warning("j is None")
                continue

            path1 = self.planj(j, obstacles=bg_object_ids + object_ids)
            if path1 is None:
                # logger.warning("path1 is None")
                continue

            self.setj(j)

            c.translate([0, 0, 0.1])
            j = self.solve_ik(
                c.pose, n_init=1, rotation_axis="z", random_state=random_state
            )
            if j is None:
                # logger.warning("j is None")
                self.setj(j_init)
                continue

            obstacles = bg_object_ids + object_ids
            obstacles.remove(object_id)
            path2 = self.planj(j, obstacles=obstacles)
            if path2 is None:
                # logger.warning("path2 is None")
                self.setj(j_init)
                continue

            self.setj(j_init)
            break
        if path1 is None or path2 is None:
            return
        for _ in (_ for j in path1 for _ in self.movej(j)):
            yield

        # XXX: getting ground truth object pose
        obj_to_world = pybullet_planning.get_pose(object_id)
        if noise:
            pos, qua = obj_to_world
            pos += random_state.normal(0, [0.003, 0.003, 0.003], 3)
            qua += random_state.normal(0, 0.01, 4)
            obj_to_world = pos, qua

        for _ in self.grasp(min_dz=0.08, max_dz=0.12, speed=0.005):
            yield

        obstacles = bg_object_ids + object_ids
        if self.gripper.check_grasp():
            obstacles.remove(object_id)
            ee_to_world = self.get_pose("tipLink")
            obj_to_ee = pybullet_planning.multiply(
                pybullet_planning.invert(ee_to_world), obj_to_world
            )
            self.attachments = [
                pybullet_planning.Attachment(
                    self.robot, self.ee, obj_to_ee, object_id
                )
            ]

            # self.virtual_grasped_object = pybullet_utils.duplicate(
            #     object_id,
            #     mass=1e-12,
            #     position=obj_to_world[0],
            #     quaternion=obj_to_world[1],
            #     rgba_color=(0, 1, 0, 0.5),
            #     texture=False,
            # )
            # p.setCollisionFilterGroupMask(
            #     self.virtual_grasped_object, -1, 0, 0
            # )
            # p.createConstraint(
            #     parentBodyUniqueId=self.robot,
            #     parentLinkIndex=self.ee,
            #     childBodyUniqueId=self.virtual_grasped_object,
            #     childLinkIndex=-1,
            #     jointType=p.JOINT_FIXED,
            #     jointAxis=(0, 0, 0),
            #     parentFramePosition=obj_to_ee[0],
            #     parentFrameOrientation=obj_to_ee[1],
            #     childFramePosition=(0, 0, 0),
            #     childFrameOrientation=(0, 0, 0, 1),
            # )
        else:
            self.attachments = []

    def move_to_homej(self, bg_object_ids, object_ids, speed=0.01, timeout=5):
        obstacles = bg_object_ids + object_ids
        if self.attachments and self.attachments[0].child in obstacles:
            obstacles.remove(self.attachments[0].child)

        js = None
        min_distance = 0
        while True:
            js = self.planj(
                self.homej,
                obstacles=obstacles,
                min_distances=utils.StaticDict(value=min_distance),
            )
            if js is not None:
                break

            if min_distance <= -0.05:
                js = [self.homej]
                break
            logger.warning(f"js is None w/ min_distance={min_distance}")
            min_distance -= 0.01
        for j in js:
            for _ in self.movej(j, speed=speed, timeout=timeout / len(js)):
                yield

    def get_cartesian_path(self, j=None, pose=None, rotation_axis=True):
        if not (j is None) ^ (pose is None):
            raise ValueError("Either j or coords must be given")

        p_start = self.get_pose("tipLink")

        with pybullet_planning.WorldSaver():
            if j is None:
                j = self.solve_ik(pose, rotation_axis=rotation_axis)
                if j is None:
                    raise RuntimeError("IK failure")
            else:
                self.setj(j)
            j_end = j

            self.setj(j_end)
            p_end = self.get_pose("tipLink")

            js_reverse = [j_end]
            for pose in pybullet_planning.interpolate_poses(p_end, p_start):
                j = self.solve_ik(
                    pose, rotation_axis=rotation_axis, validate=True
                )
                if j is None:
                    return
                js_reverse.append(j)
                self.setj(j)

        js = np.array(js_reverse[::-1])
        return js
