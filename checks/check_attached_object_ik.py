#!/usr/bin/env python

import time

import numpy as np
import pybullet as p
import pybullet_planning as pp

import reorientbot


def main():
    pp.connect(use_gui=True)
    pp.add_data_path()

    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraPitch=-20,
        cameraYaw=80,
        cameraTargetPosition=[0, 0, 0.2],
    )

    p.loadURDF("plane.urdf")
    ri = reorientbot.pybullet.PandaRobotInterface()

    # -------------------------------------------------------------------------

    cube = pp.create_box(0.03, 0.05, 0.1, mass=0.1, color=(0, 1, 0, 1))
    ee_to_world = ri.get_pose("tipLink")
    pp.draw_pose(ee_to_world)
    obj_to_ee = ([0, 0, 0.05], [0, 0, 0, 1])
    obj_to_world = pp.multiply(ee_to_world, obj_to_ee)
    p.resetBasePositionAndOrientation(cube, *obj_to_world)
    ri.gripper.contact_constraint = p.createConstraint(
        parentBodyUniqueId=ri.robot,
        parentLinkIndex=ri.ee,
        childBodyUniqueId=cube,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=(0, 0, 0),
        parentFramePosition=obj_to_ee[0],
        parentFrameOrientation=obj_to_ee[1],
        childFramePosition=(0, 0, 0),
        childFrameOrientation=(0, 0, 0, 1),
    )
    ri.gripper.activated = True

    attachments = [pp.Attachment(ri.robot, ri.ee, obj_to_ee, cube)]

    # -------------------------------------------------------------------------

    c_init = reorientbot.geometry.Coordinate(*pp.get_pose(cube))

    c1 = c_init.copy()
    c1.translate([0.3, -0.3, -0.3], wrt="world")
    c1.rotate([np.deg2rad(45), 0, 0], wrt="local")
    pp.draw_pose(c1.pose)

    c2 = c_init.copy()
    c2.translate([0.3, 0.3, -0.3], wrt="world")
    c2.rotate([np.deg2rad(-45), 0, 0], wrt="local")
    pp.draw_pose(c2.pose)

    robot_model = ri.get_skrobot(attachments=attachments)
    while True:
        joint_positions = robot_model.inverse_kinematics(
            c1.skrobot_coords,
            move_target=robot_model.attachment_link0,
        )
        for _ in ri.movej(joint_positions[:-1]):
            p.stepSimulation()
            time.sleep(1 / 240)

        joint_positions = robot_model.inverse_kinematics(
            c2.skrobot_coords,
            move_target=robot_model.attachment_link0,
        )
        for _ in ri.movej(joint_positions[:-1]):
            p.stepSimulation()
            time.sleep(1 / 240)

    pp.disconnect()


if __name__ == "__main__":
    main()
