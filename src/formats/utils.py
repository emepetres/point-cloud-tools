import numpy as np
import open3d as o3d


def create_pcd(
    points: np.array, colors: np.array, normals: bool = False
) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # compute normals
    if normals:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        # pcd.orient_normals_consistent_tangent_plane(k=15)

    return pcd
