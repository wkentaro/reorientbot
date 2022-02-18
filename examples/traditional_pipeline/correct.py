#!/usr/bin/env python

import argparse
import itertools
import json
import time

import imgviz
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
    parser.add_argument("--auc", type=float, default=0.5, help="auc")
    args = parser.parse_args()

    random_state = np.random.RandomState(args.seed)

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
        pkl_file=home / ".cache/reorientbot/pile_generation/00000001.pkl",
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
        video_dir=here / "logs/correct/video" if args.video else None,
    )
    step_simulation()

    if args.pause:
        reorientbot.pybullet.pause()

    time_table = []
    time_table_row = []

    for i_obj in itertools.count():
        i_start = step_simulation.i
        ri.homej[0] = -np.pi / 2
        for _ in ri.movej(ri.homej):
            step_simulation()
        time_table_row.append(
            ("turn_right", (step_simulation.i - i_start) * pp.get_time_step())
        )

        i_start = step_simulation.i
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
        time_table_row.append(
            ("grasp", (step_simulation.i - i_start) * pp.get_time_step())
        )
        if i == 0:
            logger.success("Completed the task")
            break

        if not ri.gripper.check_grasp():
            ri.ungrasp()
            continue

        i_start = step_simulation.i
        ri.homej[0] = 0
        for _ in ri.move_to_homej([plane, table], object_ids):
            step_simulation()
        time_table_row.append(
            ("turn_middle", (step_simulation.i - i_start) * pp.get_time_step())
        )

        i_start = step_simulation.i
        planning_time_sum = 0
        regrasp_pose = np.mean(regrasp_aabb, axis=0)
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
                        random_state=random_state,
                    ):
                        step_simulation()
                    for _ in ri.move_to_homej([plane, table], object_ids):
                        step_simulation()
                    if not ri.gripper.check_grasp():
                        ri.ungrasp()
                        continue
                    break

            t_start = time.time()
            ri.homej[0] = np.pi / 2
            with pp.LockRenderer(), pp.WorldSaver():
                ri.setj(ri.homej)
                place_pose, path = _utils.plan_placement(
                    ri, place_aabb, [plane, table], object_ids
                )
            planning_time_sum += time.time() - t_start
            if path is not None:
                break

            ri.homej[0] = 0
            regrasp_pose, _, planning_time = _utils.place_to_regrasp(
                ri,
                regrasp_aabb,
                bg_object_ids=[plane, table],
                object_ids=object_ids,
                step_simulation=step_simulation,
            )
            planning_time_sum += planning_time
        time_table_row.append(("planning", planning_time_sum))
        time_table_row.append(
            ("regrasp", (step_simulation.i - i_start) * pp.get_time_step())
        )

        i_start = step_simulation.i
        for _ in ri.move_to_homej([plane, table], object_ids):
            step_simulation()
        time_table_row.append(
            ("turn_left", (step_simulation.i - i_start) ** pp.get_time_step())
        )

        object_id = ri.attachments[0].child

        i_start = step_simulation.i
        _utils.place(
            ri,
            object_id,
            place_pose,
            path,
            bg_object_ids=[plane, table],
            object_ids=object_ids,
            step_simulation=step_simulation,
        )
        time_table_row.append(
            ("place", (step_simulation.i - i_start) * pp.get_time_step())
        )

        i_start = step_simulation.i
        _utils.correct(
            ri,
            object_id,
            place_pose,
            bg_object_ids=[plane, table],
            object_ids=object_ids,
            step_simulation=step_simulation,
            auc_threshold=args.auc,
        )
        time_table_row.append(
            ("correct", (step_simulation.i - i_start) * pp.get_time_step())
        )

        json_file = here / f"logs/correct/time_table/{i_obj}.json"
        json_file.parent.makedirs_p()
        with open(json_file, "w") as f:
            json.dump(time_table_row, f, indent=2)

        time_table.append(time_table_row)
        time_table_row = []

        plot = _utils.plot_time_table(time_table)
        imgviz.io.imsave(here / "logs/correct/time_table.jpg", plot)

    while True:
        step_simulation()


if __name__ == "__main__":
    main()
