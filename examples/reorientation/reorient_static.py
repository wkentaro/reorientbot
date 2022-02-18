#!/usr/bin/env python

import argparse
import itertools
import json
import time

from loguru import logger
import numpy as np
import path
import pybullet_planning as pp

import reorientbot

from reorientbot.examples.reorientation._env import Env
from reorientbot.examples.reorientation import _reorient
from reorientbot.examples.reorientation import _utils


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--robot-model",
        default="franka_panda/panda_suction",
        choices=["franka_panda/panda_suction", "franka_panda/panda_drl"],
        help="robot model",
    )
    parser.add_argument("--seed", type=int, help="seed", required=True)
    parser.add_argument(
        "--face",
        choices=["front", "right", "left"],
        default="front",
        help="face",
    )
    parser.add_argument("--mp4", help="mp4")
    parser.add_argument("--nogui", action="store_true", help="no gui")
    args = parser.parse_args()

    json_file = path.Path(
        f"logs/reorient_static/{args.seed:08d}-{args.face}.json"
    )
    if args.nogui and json_file.exists():
        logger.info(f"Already json_file exists: {json_file}")
        return

    env = Env(
        class_ids=[2, 3, 5, 11, 12, 15],
        robot_model=args.robot_model,
        mp4=args.mp4,
        gui=not args.nogui,
        face=args.face,
    )
    env.random_state = np.random.RandomState(args.seed)
    env.eval = True
    env.reset()

    t_start = time.time()

    reorient_poses = _reorient.get_static_reorient_poses(env)
    grasp_poses = np.array(
        list(itertools.islice(_reorient.get_grasp_poses(env), 100))
    )

    for reorient_pose in reorient_poses:
        p = np.random.permutation(grasp_poses.shape[0])[:3]
        for grasp_pose in grasp_poses[p]:
            result = _reorient.plan_reorient(env, grasp_pose, reorient_pose)
            if "js_place" in result:
                break
        if "js_place" in result:
            break

    planning_time = time.time() - t_start

    if "js_place" not in result:
        logger.error("No solution is found")
        success_reorient = False
        execution_time = np.nan
        trajectory_length = np.nan
        success = False
    else:
        exec_result = _reorient.execute_reorient(env, result)
        success_reorient = True
        execution_time = exec_result["t_place"]
        trajectory_length = result["js_place_length"]

        for _ in range(480):
            pp.step_simulation()
            if pp.has_gui():
                time.sleep(pp.get_time_step())

        pcd_in_obj, normals_in_obj = _reorient.get_query_ocs(env)
        indices = np.random.permutation(pcd_in_obj.shape[0])[:20]
        pcd_in_obj = pcd_in_obj[indices]
        normals_in_obj = normals_in_obj[indices]
        quaternion_in_obj = reorientbot.geometry.quaternion_from_vec2vec(
            [0, 0, -1], normals_in_obj
        )
        target_grasp_poses = np.hstack([pcd_in_obj, quaternion_in_obj])
        result = _reorient.plan_place(env, target_grasp_poses)
        if "js_place" not in result:
            logger.error("Failed to plan pick-and-place")
            success = False
        else:
            _reorient.execute_place(env, result)
            obj_to_world_target = env.PLACE_POSE
            obj_to_world_actual = pp.get_pose(env.fg_object_id)
            pcd_file = reorientbot.datasets.ycb.get_pcd_file(
                _utils.get_class_id(env.fg_object_id)
            )
            pcd_obj = np.loadtxt(pcd_file)
            pcd_target = reorientbot.geometry.transform_points(
                pcd_obj,
                reorientbot.geometry.transformation_matrix(*obj_to_world_target),
            )
            pcd_actual = reorientbot.geometry.transform_points(
                pcd_obj,
                reorientbot.geometry.transformation_matrix(*obj_to_world_actual),
            )
            auc = reorientbot.geometry.average_distance_auc(
                pcd_target, pcd_actual, max_threshold=0.1
            )
            success = float(auc) > 0.9

    if args.nogui:
        json_file.parent.makedirs_p()
        with open(json_file, "w") as f:
            json.dump(
                dict(
                    planning_time=planning_time,
                    success_reorient=success_reorient,
                    success=success,
                    trajectory_length=trajectory_length,
                    execution_time=execution_time,
                ),
                f,
                indent=2,
            )


if __name__ == "__main__":
    main()
