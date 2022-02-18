#!/usr/bin/env python

import pybullet as p
import pybullet_planning

import reorientbot


def main():
    pybullet_planning.connect()
    pybullet_planning.add_data_path()
    p.loadURDF("plane.urdf")

    p.resetDebugVisualizerCamera(
        cameraDistance=1,
        cameraYaw=-60,
        cameraPitch=-20,
        cameraTargetPosition=(0, 0, 0.4),
    )

    reorientbot.pybullet.create_bin(X=0.4, Y=0.6, Z=0.2)

    reorientbot.pybullet.step_and_sleep()


if __name__ == "__main__":
    main()
