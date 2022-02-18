import numpy as np
import open3d
import trimesh

import mercury


def tsdf_from_depth(depth, camera_to_base, K):
    T_camera_to_base = mercury.geometry.transformation_matrix(*camera_to_base)
    volume = open3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=0.005,
        sdf_trunc=0.04,
        color_type=open3d.pipelines.integration.TSDFVolumeColorType.Gray32,
    )
    rgbd = open3d.geometry.RGBDImage.create_from_color_and_depth(
        open3d.geometry.Image(depth),
        open3d.geometry.Image((depth * 1000).astype(np.uint16)),
        depth_trunc=1.0,
    )
    volume.integrate(
        rgbd,
        open3d.camera.PinholeCameraIntrinsic(
            width=depth.shape[1],
            height=depth.shape[0],
            fx=K[0, 0],
            fy=K[1, 1],
            cx=K[0, 2],
            cy=K[1, 2],
        ),
        np.linalg.inv(T_camera_to_base),
    )
    mesh = volume.extract_triangle_mesh()
    mesh = mesh.simplify_vertex_clustering(voxel_size=0.02)
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    mesh = trimesh.Trimesh(
        vertices=vertices,
        faces=np.r_[faces, faces[:, ::-1]],
    )
    return mesh
