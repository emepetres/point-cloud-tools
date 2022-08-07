from typing import Tuple

import laspy as lp
import numpy as np
import open3d as o3d


def _create_pcd(
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


def _get_points_colors(point_cloud: lp.LasData) -> Tuple[np.ndarray, np.ndarray]:
    points = point_cloud.xyz

    format_id = point_cloud.header.point_format.id
    hasColor = format_id == 2 or format_id == 3 or format_id == 5

    if not hasColor:
        return (points, np.full(points.shape, 255))

    colors = np.vstack(
        (point_cloud.red, point_cloud.green, point_cloud.blue)
    ).transpose()

    colors = colors / 255
    cmax = np.max(colors)

    isRGB = cmax <= 1
    isPossiblyGamma = not isRGB and cmax <= 255
    if not isRGB:
        if isPossiblyGamma:
            # assuming is gamma encoded
            colors = np.float_power(colors / 255, 2.2)
        else:
            # assuming is linear encoded
            cmin = np.min(colors)
            colors = (colors - cmin) / (cmax - cmin)

    return (points, colors)


def read_las(input: str, normals: bool = False) -> o3d.geometry.PointCloud:
    """Read a LAS/LAZ file

    Parameters
    ----------
    input: str
        input file with path
    normals:
        Set to True to compute normals. Default False

    Returns
    -------
    PointCloud
        Open3D point cloud format, optionally with normals
    """
    point_cloud = lp.read(input)
    points, colors = _get_points_colors(point_cloud)

    return _create_pcd(points, colors, normals)
