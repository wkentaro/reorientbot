#!/usr/bin/env python

import argparse
import itertools
import time

from loguru import logger
import numpy as np
import torch

import mercury

from pick_and_place_env import PickAndPlaceEnv

import common_utils
from planned import execute_plan
from planned import get_grasp_poses
from planned import get_reorient_poses
from planned import plan_and_execute_place
from planned import plan_reorient
from train import Dataset
from train import Model


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--pause", action="store_true", help="pause")
    parser.add_argument("--nolearning", action="store_true", help="nolearning")
    parser.add_argument(
        "--num-sample", type=int, default=10, help="num sample"
    )
    args = parser.parse_args()

    env = PickAndPlaceEnv()
    env.random_state = np.random.RandomState(5)
    env.reset(pile_file=env.PILES_DIR / "00001000.npz")

    common_utils.pause(args.pause)

    model = Model()
    model.cuda()
    model.load_state_dict(
        torch.load("logs/test/model_best-epoch_0219-eval_loss_0.342.pth")
    )
    model.eval()

    t_start = time.time()

    grasp_pose = []
    reorient_pose = []
    for c_reorient, c_grasp in itertools.product(
        itertools.islice(get_reorient_poses(env), 4 ** 4),
        itertools.islice(get_grasp_poses(env), 16),
    ):
        grasp_pose.append(np.concatenate(c_grasp.pose))
        reorient_pose.append(np.concatenate(c_reorient.pose))
    grasp_pose = np.stack(grasp_pose, axis=0).astype(np.float32)
    reorient_pose = np.stack(reorient_pose, axis=0).astype(np.float32)

    if args.nolearning:
        N = len(reorient_pose)
        success_pred = np.full(N, np.nan)
        length_pred = np.full(N, np.nan)
        indices = np.arange(N)
    else:
        with torch.no_grad():
            success_pred, length_pred = model(
                grasp_pose=torch.as_tensor(grasp_pose).cuda(),
                reorient_pose=torch.as_tensor(reorient_pose).cuda(),
            )
        success_pred = success_pred.cpu().numpy()
        length_pred = length_pred.cpu().numpy()
        indices = np.argsort(length_pred)

    results = []
    for index in indices[: args.num_sample]:
        if success_pred[index] < 0.6:
            continue
        logger.info(
            f"success_pred={success_pred[index]:.1%}, "
            f"length_pred={length_pred[index] * Dataset.SCALING:.2f}"
        )
        c_grasp = mercury.geometry.Coordinate(
            grasp_pose[index, :3], grasp_pose[index, 3:]
        )
        c_reorient = mercury.geometry.Coordinate(
            reorient_pose[index, :3], reorient_pose[index, 3:]
        )
        result = plan_reorient(env, c_grasp, c_reorient)
        if "js_place_length" in result:
            results.append(result)
            if args.num_sample < 0:
                break
    result = min(results, key=lambda x: x["js_place_length"])
    logger.info(f"length_true={result['js_place_length']:.2f}")

    logger.info(f"planning_time={time.time() - t_start:.2f} [s]")

    execute_plan(env, result)

    plan_and_execute_place(env)


if __name__ == "__main__":
    main()