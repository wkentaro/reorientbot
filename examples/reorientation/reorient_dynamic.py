#!/usr/bin/env python

import argparse
import itertools
import json
import time

import gdown
from loguru import logger
import numpy as np
import path
import pybullet_planning as pp
import torch

import reorientbot

from reorientbot.examples.reorientation._env import Env
from reorientbot.examples.reorientation import _reorient
from reorientbot.examples.reorientation import _utils
from reorientbot.examples.reorientation.pickable_eval import (
    get_goal_oriented_reorient_poses,  # NOQA
)
from reorientbot.examples.reorientation.reorientable_train import Model


here = path.Path(__file__).abspath().parent


models = {}

models["franka_panda/panda_drl"] = Model()
model_file = gdown.cached_download(
    id="1phRRjEqCelMo8W2O5_kYmjrmV8DSMtSe",
    path=here
    / "logs/reorientable/20210811_174552.543384-panda_drl/models/model_best-epoch_0111.pt",  # NOQA
    md5="b784e2b7c866f9475d7a38d74d321e3b",
)
models["franka_panda/panda_drl"].load_state_dict(torch.load(model_file))
models["franka_panda/panda_drl"].eval()

models["franka_panda/panda_suction"] = Model()
model_file = gdown.cached_download(
    id="1UsajylR2I0OT31jLRqMA8iNZox1zTKZw",
    path=here
    / "logs/reorientable/20210819_035217.161036-panda_suction/models/model_best-epoch_0142.pt",  # NOQA
    md5="bddb1ee74ea6015cf57fa66e11dabf95",
)
models["franka_panda/panda_suction"].load_state_dict(torch.load(model_file))
models["franka_panda/panda_suction"].eval()


def plan_dynamic_reorient(env, grasp_poses, reorient_poses, pickable):
    model = models[env._robot_model]
    model.cuda()

    obj_to_world = pp.get_pose(env.fg_object_id)
    world_to_obj = pp.invert(obj_to_world)
    grasp_poses = np.array(
        [
            np.hstack(pp.multiply(world_to_obj, np.hsplit(grasp_pose, [3])))
            for grasp_pose in grasp_poses
        ]
    )  # wrt world -> wrt obj

    # pose representation -> point-normal representation
    grasp_points = []
    for grasp_pose in grasp_poses:
        ee_to_obj = np.hsplit(grasp_pose, [3])
        grasp_point_start = ee_to_obj[0]
        grasp_point_end = reorientbot.geometry.transform_points(
            [[0, 0, 1]], reorientbot.geometry.transformation_matrix(*ee_to_obj)
        )[0]
        grasp_points.append(np.hstack([grasp_point_start, grasp_point_end]))

        if 0:
            pp.draw_pose(
                np.hsplit(grasp_pose, [3]),
                parent=env.fg_object_id,
                length=0.05,
                width=3,
            )
    grasp_points = np.array(grasp_points)

    N_grasp = grasp_points.shape[0]
    N_reorient = reorient_poses.shape[0]
    B = N_grasp * N_reorient
    logger.info(f"N_grasp: {N_grasp}, N_reorient: {N_reorient}, B: {B}")

    grasp_poses = grasp_poses[:, None, :].repeat(N_reorient, axis=1)
    grasp_points = grasp_points[:, None, :].repeat(N_reorient, axis=1)
    reorient_poses = reorient_poses[None, :, :].repeat(N_grasp, axis=0)
    pickable = pickable[None, :].repeat(N_grasp, axis=0)

    grasp_poses = grasp_poses.reshape(B, -1).astype(np.float32)
    grasp_points = grasp_points.reshape(B, -1).astype(np.float32)
    reorient_poses = reorient_poses.reshape(B, -1).astype(np.float32)
    pickable = pickable.reshape(B).astype(np.float32)

    class_ids = [2, 3, 5, 11, 12, 15, 16]

    heightmap = env.obs["pointmap"][:, :, 2]

    object_fg_flags = []
    object_labels = []
    object_poses = []
    for object_id in env.object_ids:
        class_id = _utils.get_class_id(object_id)
        if class_id not in class_ids:
            continue
        object_fg_flags.append(object_id == env.fg_object_id)
        object_label = np.zeros(7)
        object_label[class_ids.index(class_id)] = 1
        object_labels.append(object_label)
        object_poses.append(np.hstack(pp.get_pose(object_id)))
    object_fg_flags = np.stack(object_fg_flags, axis=0).astype(np.float32)
    object_labels = np.stack(object_labels, axis=0).astype(np.float32)
    object_poses = np.stack(object_poses, axis=0).astype(np.float32)

    object_label = object_labels[object_fg_flags == 1][0]
    object_pose = object_poses[object_fg_flags == 1][0]

    if getattr(env, "reverse", False):
        object_pose[:3] = env.PILE_POSITION + [0, -0.4, 0.1]

    with torch.no_grad():
        reorientable_pred, trajectory_length_pred = model(
            heightmap=torch.as_tensor(heightmap[None, None]).float().cuda(),
            object_label=torch.as_tensor(object_label[None]).float().cuda(),
            object_pose=torch.as_tensor(object_pose[None]).float().cuda(),
            grasp_pose=torch.as_tensor(grasp_points[None]).cuda(),
            reorient_pose=torch.as_tensor(reorient_poses[None]).cuda(),
        )
    reorientable_pred = reorientable_pred.cpu().numpy()
    trajectory_length_pred = trajectory_length_pred.cpu().numpy()

    reorientable_pred = reorientable_pred[0]
    trajectory_length_pred = trajectory_length_pred[0]

    grasp_poses = grasp_poses.reshape(N_grasp, N_reorient, 7)
    reorient_poses = reorient_poses.reshape(N_grasp, N_reorient, 7)
    pickable = pickable.reshape(N_grasp, N_reorient)
    reorientable_pred = reorientable_pred.reshape(N_grasp, N_reorient, 3)
    trajectory_length_pred = trajectory_length_pred.reshape(
        N_grasp, N_reorient
    )

    reorientable_pred = np.prod(reorientable_pred, axis=2)

    print(reorientable_pred.max())

    N_top = 3
    i_grasp = np.arange(N_grasp)[:, None].repeat(N_top, axis=1)
    i_reorient = np.argsort(reorientable_pred, axis=1)[:, -N_top:]

    pickable = pickable[i_grasp, i_reorient]
    reorientable_pred = reorientable_pred[i_grasp, i_reorient]
    trajectory_length_pred = trajectory_length_pred[i_grasp, i_reorient]
    grasp_poses = grasp_poses[i_grasp, i_reorient]
    reorient_poses = reorient_poses[i_grasp, i_reorient]

    pickable = pickable.reshape(N_grasp * N_top)
    reorientable_pred = reorientable_pred.reshape(N_grasp * N_top)
    trajectory_length_pred = trajectory_length_pred.reshape(N_grasp * N_top)
    grasp_poses = grasp_poses.reshape(N_grasp * N_top, 7)
    reorient_poses = reorient_poses.reshape(N_grasp * N_top, 7)

    if 0:
        indices1 = np.argsort(trajectory_length_pred)
        indices2 = np.argsort(reorientable_pred)[::-1]
        indices = np.r_[indices1[:3], indices2[:3], indices1[3:], indices2[3:]]
    else:
        indices = np.argsort(reorientable_pred)[::-1]

    if _utils.get_class_id(env.fg_object_id) == 2:
        keep = reorientable_pred[indices] > 0.3
        indices = indices[keep]

    assert (
        pickable.shape[0]
        == reorientable_pred.shape[0]
        == trajectory_length_pred.shape[0]
        == grasp_poses.shape[0]
        == reorient_poses.shape[0]
    )

    result = {}
    for index in indices:
        ee_to_obj = np.hsplit(grasp_poses[index], [3])
        ee_to_world = pp.multiply(obj_to_world, ee_to_obj)
        obj_af_to_world = np.hsplit(reorient_poses[index], [3])

        result = _reorient.plan_reorient(
            env, np.hstack(ee_to_world), np.hstack(obj_af_to_world)
        )

        if _utils.get_class_id(env.fg_object_id) == 11:
            if "js_place_length" in result and result["js_place_length"] > 4.5:
                result.pop("js_place")
                result.pop("js_place_length")

        if "js_place" in result:
            logger.success(
                f"pickable={pickable[index]:.1%}, "
                # f"graspable_pred={reorientable_pred[index, 0]:.1%}, "
                # f"placable_pred={reorientable_pred[index, 1]:.1%}, "
                # f"reorientable_pred={reorientable_pred[index, 2]:.1%}, "
                f"reorientable_pred={reorientable_pred[index]:.1%}, "
                f"trajectory_length_pred={trajectory_length_pred[index]:.1f}, "
                f"trajectory_length_true={result['js_place_length']:.1f}"
            )
            break
        else:
            logger.warning(
                f"pickable={pickable[index]:.1%}, "
                # f"graspable_pred={reorientable_pred[index, 0]:.1%}, "
                # f"placable_pred={reorientable_pred[index, 1]:.1%}, "
                # f"reorientable_pred={reorientable_pred[index, 2]:.1%}, "
                f"reorientable_pred={reorientable_pred[index]:.1%}, "
                f"trajectory_length_pred={trajectory_length_pred[index]:.1f}"
            )

    return result


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
    parser.add_argument("--nodebug", action="store_true", help="no debug")
    args = parser.parse_args()

    json_file = path.Path(
        f"logs/reorient_dynamic/{args.seed:08d}-{args.face}.json"
    )
    if args.nogui and json_file.exists():
        logger.info(f"Already json_file exists: {json_file}")
        return

    env = Env(
        class_ids=[2, 3, 5, 11, 12, 15],
        robot_model=args.robot_model,
        gui=not args.nogui,
        mp4=args.mp4,
        face=args.face,
        debug=not args.nodebug,
    )
    env.random_state = np.random.RandomState(args.seed)
    env.eval = True
    env.reset()

    t_start = time.time()

    (
        reorient_poses,
        pickable,
        target_grasp_poses,
    ) = get_goal_oriented_reorient_poses(env)

    grasp_poses = np.array(
        list(itertools.islice(_reorient.get_grasp_poses(env), 32))
    )
    if 0:
        for reorient_pose in reorient_poses[np.argsort(pickable)[::-1]]:
            grasp_pose = grasp_poses[
                np.random.permutation(grasp_poses.shape[0])[0]
            ]
            result = _reorient.plan_reorient(env, grasp_pose, reorient_pose)
            if "js_place" in result:
                break
        _reorient.execute_reorient(env, result)
    else:
        for threshold in np.linspace(0.9, 0.1, num=10):
            indices = np.where(pickable > threshold)[0]
            if indices.size > 100:
                break
        indices = np.random.choice(
            indices, min(indices.size, 1000), replace=False
        )
        reorient_poses = reorient_poses[indices]
        pickable = pickable[indices]

        result = plan_dynamic_reorient(
            env, grasp_poses, reorient_poses, pickable
        )

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
