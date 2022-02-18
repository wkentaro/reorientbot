#!/usr/bin/env python

import argparse
import itertools

from loguru import logger
import numpy as np
import path
import pybullet_planning as pp

import reorientbot

import _utils


here = path.Path(__file__).abspath().parent
home = path.Path("~").expanduser()


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--pause", action="store_true", help="pause")
    parser.add_argument(
        "--camera-config", type=int, default=0, help="camera config"
    )
    parser.add_argument("--video", action="store_true", help="video")
    parser.add_argument("--seed", type=int, default=0, help="seed")
    parser.add_argument("--retime", type=float, default=1, help="retime")
    args = parser.parse_args()

    np.random.seed(args.seed)

    pp.connect()
    plane = _utils.init_simulation(camera_distance=1.2)

    ri = reorientbot.pybullet.PandaRobotInterface()
    ri.add_camera(
        pose=_utils.get_camera_pose(args.camera_config),
        height=240,
        width=320,
    )

    pile_pose = ([0, -0.5, 0], [0, 0, 0, 1])
    object_ids = _utils.load_pile(
        base_pose=pile_pose,
        pkl_file=home / "data/reorientbot/pile_generation/00000001.pkl",
        mass=0.1,
    )
    for obj in object_ids:
        pp.draw_pose(
            ([0, 0, 0], [0, 0, 0, 1]), length=0.15, width=3, parent=obj
        )

    table = pp.create_box(
        0.4, 0.4, 0.1, color=[150 / 255, 111 / 255, 51 / 255, 1]
    )
    pp.set_pose(table, ([0.5, 0, 0.1], [0, 0, 0, 1]))
    aabb = pp.get_aabb(table)
    regrasp_aabb = (
        [aabb[0][0] + 0.1, aabb[0][1] + 0.1, aabb[1][2]],
        [aabb[1][0] - 0.1, aabb[1][1] - 0.1, aabb[1][2] + 0.001],
    )
    pp.draw_aabb(regrasp_aabb)

    place_aabb = ((-0.3, 0.3, 0), (0.3, 0.6, 0.2))
    pp.draw_aabb(place_aabb, width=2)

    step_simulation = _utils.StepSimulation(
        ri=ri,
        retime=args.retime,
        video_dir=here / "logs/regrasp/video" if args.video else None,
    )
    step_simulation()

    if args.pause:
        reorientbot.pybullet.pause()

    while True:
        ri.homej[0] = -np.pi / 2
        for _ in ri.movej(ri.homej):
            step_simulation()

        c = reorientbot.geometry.Coordinate(*ri.get_pose("camera_link"))
        c.position = pile_pose[0]
        c.position[2] = 0.7
        c.quaternion = reorientbot.geometry.quaternion_from_euler(
            [np.pi, 0, np.pi / 2]
        )
        j = ri.solve_ik(c.pose, move_target=ri.robot_model.camera_link)
        for _ in ri.movej(j):
            step_simulation()

        i = 0
        _, depth, segm = ri.get_camera_image()
        for _ in ri.random_grasp(
            depth,
            segm,
            mask=np.isin(segm, object_ids),
            bg_object_ids=[plane, table],
            object_ids=object_ids,
        ):
            step_simulation()
            i += 1
        for _ in ri.move_to_homej(
            bg_object_ids=[plane, table], object_ids=object_ids
        ):
            step_simulation()
        if i == 0:
            logger.success("Completed the task")
            break

        if not ri.gripper.check_grasp():
            ri.ungrasp()
            continue

        ri.homej[0] = 0
        for _ in ri.move_to_homej([plane, table], object_ids):
            step_simulation()

        regrasp_pose = np.mean(regrasp_aabb, axis=0)
        done = False
        for i in itertools.count():
            if i > 0:
                while True:
                    c = reorientbot.geometry.Coordinate(*ri.get_pose("tipLink"))
                    c.position = regrasp_pose[0]
                    c.position[2] = 0.7
                    j = ri.solve_ik(
                        c.pose,
                        move_target=ri.robot_model.camera_link,
                        rotation_axis="z",
                    )
                    for _ in ri.movej(j):
                        step_simulation()
                    _, depth, segm = ri.get_camera_image()
                    for _ in ri.random_grasp(
                        depth,
                        segm,
                        mask=np.isin(segm, object_ids),
                        bg_object_ids=[plane, table],
                        object_ids=object_ids,
                        max_angle=np.deg2rad(10),
                    ):
                        step_simulation()
                    for _ in ri.move_to_homej([plane, table], object_ids):
                        step_simulation()
                    if not ri.gripper.check_grasp():
                        ri.ungrasp()
                        continue

                    ri.homej[0] = np.pi / 2
                    with pp.LockRenderer(), pp.WorldSaver():
                        ri.setj(ri.homej)
                        place_pose, path = _utils.plan_placement(
                            ri, place_aabb, [plane, table], object_ids
                        )
                    if path is None:
                        done = False
                        break

                    break

                if done:
                    break

            ri.homej[0] = 0
            regrasp_pose, done, _ = _utils.place_to_regrasp(
                ri,
                regrasp_aabb,
                bg_object_ids=[plane, table],
                object_ids=object_ids,
                step_simulation=step_simulation,
            )

        object_id = ri.attachments[0].child

        pp.remove_body(object_id)
        ri.ungrasp()

    while True:
        step_simulation()


if __name__ == "__main__":
    main()
