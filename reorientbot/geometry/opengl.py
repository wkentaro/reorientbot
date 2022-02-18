import numpy as np
import trimesh


def opengl_intrinsic_matrix(fovy, height, width):
    aspect_ratio = width / height
    fovx = 2 * np.arctan(np.tan(fovy * 0.5) * aspect_ratio)
    return trimesh.scene.Camera(
        resolution=(width, height),
        fov=(np.rad2deg(fovx), np.rad2deg(fovy)),
    ).K


def to_opengl_transform(transform=None):
    if transform is None:
        transform = np.eye(4)
    return transform @ trimesh.transformations.rotation_matrix(
        np.deg2rad(-180), [1, 0, 0]
    )


def from_opengl_transform(transform=None):
    if transform is None:
        transform = np.eye(4)
    return transform @ trimesh.transformations.rotation_matrix(
        np.deg2rad(180), [1, 0, 0]
    )
