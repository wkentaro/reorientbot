import argparse

import pybullet as p
import pybullet_planning


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("urdf_file", help="urdf file")
    args = parser.parse_args()

    pybullet_planning.connect()

    with pybullet_planning.LockRenderer():
        body = p.loadURDF(args.urdf_file)

    aabb = pybullet_planning.get_aabb(body)
    pybullet_planning.draw_aabb(aabb)

    aabb_extent = pybullet_planning.get_aabb_extent(aabb)
    aabb_center = pybullet_planning.get_aabb_center(aabb)
    p.resetDebugVisualizerCamera(
        cameraTargetPosition=aabb_center,
        cameraDistance=max(aabb_extent),
        cameraYaw=45,
        cameraPitch=-30,
    )

    while True:
        try:
            p.stepSimulation()
        except p.error:
            break
