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

    plane = p.loadURDF("plane.urdf")
    ri = reorientbot.pybullet.PandaRobotInterface()

    # -------------------------------------------------------------------------

    box = pp.create_box(0.7, 0.1, 0.4)
    p.resetBasePositionAndOrientation(box, [0.5, 0, 0.2], [0, 0, 0, 1])

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
    ri.attachments = [pp.Attachment(ri.robot, ri.ee, obj_to_ee, cube)]

    # -------------------------------------------------------------------------

    c1 = reorientbot.geometry.Coordinate(*pp.get_link_pose(ri.robot, ri.ee))
    c1.translate([0.4, -0.4, -0.4], wrt="world")

    c2 = c1.copy()
    c2.translate([0, 0.8, 0], wrt="world")

    obstacles = [plane, box]

    j1 = ri.solve_ik(c1.pose)
    j2 = ri.solve_ik(c2.pose)

    while True:
        traj = None
        while traj is None:
            traj = ri.planj(j1, obstacles=obstacles)
        for j in traj:
            for _ in ri.movej(j):
                p.stepSimulation()
                time.sleep(1 / 240)

        traj = None
        while traj is None:
            traj = ri.planj(j2, obstacles=obstacles)
        for j in traj:
            for _ in ri.movej(j):
                p.stepSimulation()
                time.sleep(1 / 240)

    pp.disconnect()


if __name__ == "__main__":
    main()
