#!/usr/bin/env python

import time

import pybullet as p
import pybullet_planning as pp

import reorientbot


def main():
    pp.connect(use_gui=True)
    pp.add_data_path()

    p.resetDebugVisualizerCamera(
        cameraDistance=2,
        cameraPitch=-20,
        cameraYaw=80,
        cameraTargetPosition=[0, 0, 0],
    )

    p.loadURDF("plane.urdf")
    ri = reorientbot.pybullet.PandaRobotInterface()

    # -------------------------------------------------------------------------

    cube = pp.create_box(0.05, 0.05, 0.05, mass=0.1, color=(0, 1, 0, 1))
    ee_to_world = ri.get_pose("tipLink")
    obj_to_ee = ([0, 0, 0.025], [0, 0, 0, 1])
    obj_to_world = pp.multiply(ee_to_world, obj_to_ee)
    p.resetBasePositionAndOrientation(cube, *obj_to_world)
    p.createConstraint(
        parentBodyUniqueId=ri.robot,
        parentLinkIndex=ri.ee,
        childBodyUniqueId=cube,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=(0, 0, 0),
        parentFramePosition=obj_to_ee[0],
        parentFrameOrientation=obj_to_ee[1],
        childFramePosition=(0, 0, 0),
    )

    # -------------------------------------------------------------------------

    coord = reorientbot.geometry.Coordinate(*pp.get_link_pose(ri.robot, ri.ee))
    coord.translate([0.5, 0, -0.5], wrt="world")

    robot_model = ri.get_skrobot()
    while True:
        joint_positions = robot_model.inverse_kinematics(
            coord.skrobot_coords, move_target=robot_model.tipLink
        )
        for _ in ri.movej(joint_positions):
            p.stepSimulation()
            time.sleep(1 / 240)
        for _ in ri.movej(ri.homej):
            p.stepSimulation()
            time.sleep(1 / 240)

    pp.disconnect()


if __name__ == "__main__":
    main()
