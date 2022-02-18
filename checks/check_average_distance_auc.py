#!/usr/bin/env python

import numpy as np
import trimesh

import reorientbot


def main():
    pcd_file = reorientbot.datasets.ycb.get_pcd_file(class_id=1)
    pcd = trimesh.PointCloud(vertices=np.loadtxt(pcd_file))

    position = np.random.uniform(-0.5, 0.5, (3,))
    quaternion = np.random.uniform(-1, 1, (4,))
    quaternion /= np.linalg.norm(quaternion)
    T_obj_to_true = reorientbot.geometry.transformation_matrix(
        position, quaternion
    )
    pcd_true = pcd.copy()
    pcd_true.apply_transform(T_obj_to_true)
    pcd_true.colors = [0, 1.0, 0]

    pcd_pred = pcd.copy()
    position += np.random.normal(0, (0.01 / 3,) * 3)
    quaternion += np.random.normal(0, (0.09 / 3,) * 4)
    T_obj_to_pred = reorientbot.geometry.transformation_matrix(
        position, quaternion
    )
    pcd_pred.apply_transform(T_obj_to_pred)
    pcd_pred.colors = [1.0, 0, 0]

    auc = reorientbot.geometry.average_distance_auc(
        reference=pcd_true.vertices, query=pcd_pred.vertices, plot=True
    )
    print(auc)

    trimesh.Scene([pcd_true, pcd_pred]).show()


if __name__ == "__main__":
    main()
