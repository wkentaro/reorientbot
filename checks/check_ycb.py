#!/usr/bin/env python

import argparse

import imgviz
import numpy as np
import pybullet as p
import pybullet_planning
import tqdm

import reorientbot


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--type", choices=["all", "selected"], default="all", help="type"
    )
    args = parser.parse_args()

    pybullet_planning.connect(use_gui=False)

    if args.type == "selected":
        # selected
        class_ids = [1, 2, 3, 4, 5, 7, 8, 9, 11, 12, 13, 16]
    elif args.type == "all":
        # all
        class_ids = list(range(1, 22))
    else:
        raise ValueError

    tiled = []
    class_names = reorientbot.datasets.ycb.class_names
    for class_id in tqdm.tqdm(class_ids):
        visual_file = reorientbot.datasets.ycb.get_visual_file(class_id)
        obj = reorientbot.pybullet.create_mesh_body(
            visual_file=visual_file,
        )

        T_camera_to_world = reorientbot.geometry.look_at(
            eye=[0.2, 0.2, 0.2], target=[0, 0, 0]
        )
        rgb, _, _ = reorientbot.pybullet.get_camera_image(
            T_cam2world=T_camera_to_world,
            fovy=np.deg2rad(50),
            height=230,
            width=250,
        )
        text = f"{class_id:02d}: {class_names[class_id]}"
        rgb = imgviz.draw.text_in_rectangle(
            rgb,
            text=text,
            loc="lt+",
            size=15,
            background=(255, 255, 255),
            color=(0, 0, 0),
        )
        tiled.append(rgb)

        p.removeBody(obj)

    pybullet_planning.disconnect()

    tiled = imgviz.tile(
        tiled, shape=(-1, 4), cval=(255, 255, 255), border=(0, 0, 0)
    )
    out_file = "/tmp/ycb.jpg"
    imgviz.io.imsave(out_file, tiled)
    print(f"==> Saved to {out_file}")


if __name__ == "__main__":
    main()
