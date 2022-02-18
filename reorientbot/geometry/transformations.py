import numpy as np
import trimesh.transformations as ttf
from trimesh.transformations import angle_between_vectors  # NOQA
from trimesh.transformations import transform_around  # NOQA
from trimesh.transformations import translation_from_matrix  # NOQA
from trimesh.transformations import translation_matrix  # NOQA


def transform_points(points, matrix, translate=True):
    points = np.asarray(points)
    return ttf.transform_points(
        points.reshape(-1, 3), matrix, translate=translate
    ).reshape(points.shape)


def quaternion_from_matrix(matrix):
    q = np.empty((4,), dtype=matrix.dtype)
    t = np.trace(matrix)
    if t > matrix[3, 3]:
        q[3] = t
        q[2] = matrix[1, 0] - matrix[0, 1]
        q[1] = matrix[0, 2] - matrix[2, 0]
        q[0] = matrix[2, 1] - matrix[1, 2]
    else:
        i, j, k = 0, 1, 2
        if matrix[1, 1] > matrix[0, 0]:
            i, j, k = 1, 2, 0
        if matrix[2, 2] > matrix[i, i]:
            i, j, k = 2, 0, 1
        t = matrix[i, i] - (matrix[j, j] + matrix[k, k]) + matrix[3, 3]
        q[i] = t
        q[j] = matrix[i, j] + matrix[j, i]
        q[k] = matrix[k, i] + matrix[i, k]
        q[3] = matrix[k, j] - matrix[j, k]
    q *= 0.5 / np.sqrt(t * matrix[3, 3])
    return q


def quaternion_matrix(quaternion):
    quaternion = np.asarray(quaternion)[[3, 0, 1, 2]]  # xyzw -> wxyz
    return ttf.quaternion_matrix(quaternion)


def quaternion_from_euler(euler):
    quaternion = ttf.quaternion_from_euler(*euler, axes="rxyz")
    if quaternion[0] < 0.0:
        # make w is always positive
        np.negative(quaternion, quaternion)
    quaternion = quaternion[[1, 2, 3, 0]]  # wxyz -> xyzw
    return quaternion


def euler_from_quaternion(quaternion):
    quaternion = np.asarray(quaternion)[[3, 0, 1, 2]]  # xyzw -> wxyz
    euler = ttf.euler_from_quaternion(quaternion, axes="rxyz")
    euler = np.array(euler, dtype=np.float64)
    return euler


def euler_matrix(euler):
    ai, aj, ak = euler
    return ttf.euler_matrix(ai, aj, ak, axes="rxyz")


def euler_from_matrix(matrix):
    return ttf.euler_from_matrix(matrix, axes="rxyz")


def transformation_matrix(translation, quaternion):
    matrix = quaternion_matrix(quaternion)
    matrix[:3, 3] = translation
    return matrix


def pose_from_matrix(matrix):
    translation = translation_from_matrix(matrix)
    quaternion = quaternion_from_matrix(matrix)
    return translation, quaternion
