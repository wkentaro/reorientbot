#!/usr/bin/env python

import argparse
import time

import numpy as np
import pybullet as p
import pybullet_planning as pp

import reorientbot

import _utils


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seed", type=int, default=1, help="random seed")
    args = parser.parse_args()

    pp.connect()
    _utils.init_simulation(camera_distance=1)

    unique_ids = _utils.create_pile(
        class_ids=[2, 3, 5, 11, 12, 15, 16],
        num_instances=8,
        random_state=np.random.RandomState(args.seed),
    )

    for unique_id in unique_ids:
        class_id = int(p.getUserData(p.getUserDataId(unique_id, "class_id")))
        class_name = reorientbot.datasets.ycb.class_names[class_id]
        print(
            f"body_id={unique_id}, class_id={class_id:02d}, "
            f"class_name={class_name}"
        )

    while True:
        p.stepSimulation()
        time.sleep(1 / 240)


if __name__ == "__main__":
    main()
