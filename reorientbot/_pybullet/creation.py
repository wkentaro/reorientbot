import numpy as np
import pybullet as p


def create_bin(X, Y, Z, color=(0.59, 0.44, 0.2, 1), create=None):
    origin = [0, 0, 0]

    if create is None:
        create = Ellipsis

    def get_parts(origin, X, Y, Z, T=0.01):
        extents = np.array(
            [
                [X, Y, T],
                [X, T, Z],
                [X, T, Z],
                [T, Y, Z],
                [T, Y, Z],
            ]
        )[create]
        positions = np.array(
            [
                [0, 0, -Z / 2],
                [0, Y / 2, 0],
                [0, -Y / 2, 0],
                [X / 2, 0, 0],
                [-X / 2, 0, 0],
            ]
        )[create]
        positions += origin
        return extents, positions

    extents, positions = get_parts(origin, X, Y, Z)

    halfExtents = np.array(extents) / 2
    shapeTypes = [p.GEOM_BOX] * len(extents)
    rgbaColors = [color] * len(extents)
    visual_shape_id = p.createVisualShapeArray(
        shapeTypes=shapeTypes,
        halfExtents=halfExtents,
        visualFramePositions=positions,
        rgbaColors=rgbaColors,
    )
    collision_shape_id = p.createCollisionShapeArray(
        shapeTypes=shapeTypes,
        halfExtents=halfExtents,
        collisionFramePositions=positions,
    )

    position = [0, 0, Z / 2]
    quaternion = [0, 0, 0, 1]
    unique_id = p.createMultiBody(
        baseMass=0,
        basePosition=position,
        baseOrientation=quaternion,
        baseVisualShapeIndex=visual_shape_id,
        baseCollisionShapeIndex=collision_shape_id,
        baseInertialFramePosition=[0, 0, 0],
        baseInertialFrameOrientation=[0, 0, 0, 1],
    )
    return unique_id
