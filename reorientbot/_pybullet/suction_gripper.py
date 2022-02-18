from loguru import logger
import numpy as np
import pybullet as p
import pybullet_planning

from .. import geometry


class SuctionGripper:
    def __init__(
        self,
        body,
        link,
        max_force=10,
        surface_threshold=np.deg2rad(10),
        surface_alignment=True,
    ):
        self.body = body
        self.link = link
        self.max_force = max_force

        self._surface_threshold = surface_threshold
        self._surface_alignment = surface_alignment

        self.activated = False
        self.contact_constraint = None
        self.grasp_point_on_obj = True
        self.grasp_point_on_ee = None

    def activate(self):
        if self.activated:
            raise RuntimeError("suction gripper is already activated")

        self.activated = True

        points = p.getContactPoints(bodyA=self.body, linkIndexA=self.link)
        if not points:
            logger.warning("suction gripper didn't contact any surface")
            return

        # Handle contact between suction with a rigid object.
        point = points[-1]

        obj_id = point[2]
        contact_link = point[4]
        contact_distance = point[8]

        # in world coordinates
        point_on_ee = point[5]
        point_on_obj = point[6]

        # in obj coordinates
        obj_to_world = pybullet_planning.get_pose(point[2])
        T_world_to_obj = geometry.transformation_matrix(
            *pybullet_planning.invert(obj_to_world)
        )
        grasp_point_on_obj = geometry.transform_points(
            [point_on_obj], T_world_to_obj
        )[0]

        # in ee coordinates
        ee_to_world = pybullet_planning.get_link_pose(self.body, self.link)
        world_to_ee = pybullet_planning.invert(ee_to_world)
        T_world_to_ee = geometry.transformation_matrix(*world_to_ee)
        point_on_ee = geometry.transform_points([point_on_ee], T_world_to_ee)[
            0
        ]
        point_on_obj = geometry.transform_points(
            [point_on_obj], T_world_to_ee
        )[0]
        grasp_point_on_ee = point_on_ee

        v_ee_to_obj = (point_on_obj - point_on_ee) * np.sign(contact_distance)
        v_ee_to_obj /= np.linalg.norm(v_ee_to_obj)

        angle = np.abs(np.arccos(np.dot(v_ee_to_obj, [0, 0, 1])))

        mass = p.getDynamicsInfo(obj_id, -1)[0]
        if mass == 0:
            logger.warning("object in contact is not dynamic")
            return

        if angle > self._surface_threshold:
            logger.warning(
                "failed to grasp with surface angle "
                f">{np.rad2deg(self._surface_threshold):.1f} deg: "
                f"{np.rad2deg(angle):.1f} deg"
            )
            return
        logger.info(f"grasping surface angle: {np.rad2deg(angle):.1f} deg")

        # simulate compliance of suction gripper
        T_obj_to_obj_af_in_ee = geometry.transformation_matrix(
            point_on_ee - point_on_obj,
            geometry.quaternion_from_vec2vec(v_ee_to_obj, [0, 0, 1]),
        )
        T_obj_to_obj_af_in_ee = geometry.transform_around(
            T_obj_to_obj_af_in_ee, point_on_obj
        )
        T_obj_to_world = geometry.transformation_matrix(
            *pybullet_planning.get_pose(obj_id)
        )
        T_obj_to_ee = T_world_to_ee @ T_obj_to_world
        T_obj_af_to_ee = T_obj_to_obj_af_in_ee @ T_obj_to_ee
        T_obj_af_to_world = np.linalg.inv(T_world_to_ee) @ T_obj_af_to_ee

        ee_to_world = p.getLinkState(self.body, self.link)[:2]
        if self._surface_alignment:  # w/ compliance
            obj_to_world = geometry.pose_from_matrix(T_obj_af_to_world)
        else:  # w/o compliance
            obj_to_world = p.getBasePositionAndOrientation(obj_id)
        world_to_ee = pybullet_planning.invert(ee_to_world)
        obj_to_ee = pybullet_planning.multiply(world_to_ee, obj_to_world)
        self.add_constraint(
            obj=obj_id,
            obj_link=contact_link,
            obj_to_ee=obj_to_ee,
            grasp_point_on_obj=grasp_point_on_obj,
            grasp_point_on_ee=grasp_point_on_ee,
        )

    def add_constraint(
        self,
        obj,
        obj_to_ee,
        obj_link=-1,
        grasp_point_on_obj=None,
        grasp_point_on_ee=None,
    ):
        self.grasp_point_on_obj = grasp_point_on_obj
        self.grasp_point_on_ee = grasp_point_on_ee
        self.contact_constraint = p.createConstraint(
            parentBodyUniqueId=self.body,
            parentLinkIndex=self.link,
            childBodyUniqueId=obj,
            childLinkIndex=obj_link,
            jointType=p.JOINT_FIXED,
            jointAxis=(0, 0, 0),
            parentFramePosition=obj_to_ee[0],
            parentFrameOrientation=obj_to_ee[1],
            childFramePosition=(0, 0, 0),
            childFrameOrientation=(0, 0, 0),
        )
        if self.max_force is not None:
            p.changeConstraint(
                self.contact_constraint, maxForce=self.max_force
            )

    def step_simulation(self):
        # this function must be called after p.stepSimulation()
        if self.grasped_object is not None:
            if (
                self.grasp_point_on_obj is None
                or self.grasp_point_on_ee is None
            ):
                logger.warning(
                    "self.grasp_point_on_obj or self.grasp_point_on_ee"
                    "is not set"
                )
                return
            obj_to_world = pybullet_planning.get_pose(self.grasped_object)
            grasp_point_on_obj = geometry.transform_points(
                [self.grasp_point_on_obj],
                geometry.transformation_matrix(*obj_to_world),
            )[0]
            ee_to_world = pybullet_planning.get_link_pose(self.body, self.link)
            grasp_point_on_ee = geometry.transform_points(
                [self.grasp_point_on_ee],
                geometry.transformation_matrix(*ee_to_world),
            )[0]

            distance = np.linalg.norm(grasp_point_on_ee - grasp_point_on_obj)
            if distance > 0.01:
                # surface is apart more than 1cm
                logger.warning("dropping grasped object as surfaces are apart")
                if self.contact_constraint is not None:
                    try:
                        p.removeConstraint(self.contact_constraint)
                        self.contact_constraint = None
                    except Exception:
                        pass

    def release(self):
        if not self.activated:
            raise RuntimeError("suction gripper is not yet activated")

        self.activated = False

        # Release gripped rigid object (if any).
        if self.contact_constraint is not None:
            try:
                p.removeConstraint(self.contact_constraint)
                self.contact_constraint = None
            except Exception:
                pass

    def detect_contact(self):
        body, link = self.body, self.link
        if self.activated and self.contact_constraint is not None:
            try:
                info = p.getConstraintInfo(self.contact_constraint)
                body, link = info[2], info[3]
            except Exception:
                self.contact_constraint = None
                pass

        # Get all contact points between the suction and a rigid body.
        points = p.getContactPoints(bodyA=body, linkIndexA=link)
        if self.activated:
            points = [point for point in points if point[2] != self.body]

        # We know if len(points) > 0, contact is made with SOME rigid item.
        if points:
            return True

        return False

    def check_grasp(self):
        return self.grasped_object is not None

    @property
    def grasped_object(self):
        grasped_object = None
        if self.contact_constraint is not None:
            grasped_object = p.getConstraintInfo(self.contact_constraint)[2]
        return grasped_object
