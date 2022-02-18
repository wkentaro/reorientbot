#!/usr/bin/env python

import json

import numpy as np
import pandas
import path


df_data = []

HEIGHTMAP_PIXEL_SIZE = 0.004  # 4mm
DIFF_THRESHOLD = 0.01  # 1cm

logs_dir = path.Path("logs")
for scene_dir in logs_dir.listdir():
    for episode_dir in scene_dir.listdir():
        heightmap1 = np.load(episode_dir / "heightmap1.npy")
        heightmap2 = np.load(episode_dir / "heightmap2.npy")

        diff = abs(heightmap1 - heightmap2)
        diff_mask = diff > DIFF_THRESHOLD
        diff_volume = np.nansum(HEIGHTMAP_PIXEL_SIZE ** 2 * diff)

        with open(episode_dir / "data.json") as f:
            data = json.load(f)
        df_data.append(
            dict(
                scene_id=str(scene_dir.stem),
                model=data["model"],
                diff_mask_ratio=diff_mask.mean(),
                diff_volume=diff_volume * 100 ** 3 / 1000,  # [l]
            )
        )


df = pandas.DataFrame(df_data)
df = df.sort_values(["scene_id", "model"])
print(df)
print(df.groupby("model").mean())
