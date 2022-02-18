import numpy as np


def quaternion_from_vec2vec(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)

    assert v1.ndim == 1
    v2_ndim = v2.ndim
    if v2_ndim == 1:
        v2 = v2[None]

    xyz = np.cross(v1, v2)
    w = np.sqrt((v1 ** 2).sum() * (v2 ** 2).sum(axis=1)) + np.dot(v1, v2.T)
    q = np.hstack([xyz, w[:, None]])

    if v2_ndim == 1:
        q = q[0]

    return q
