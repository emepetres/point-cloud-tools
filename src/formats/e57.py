from typing import Tuple

import pye57
import numpy as np
import open3d as o3d

from formats.utils import create_pcd


def _get_points_colors(e57: pye57.E57) -> Tuple[np.ndarray, np.ndarray]:
    header = e57.get_header(0)
    if "cartesianX" not in header.point_fields:
        raise ValueError("E57 file does not have cartesian information")
    hasColor = "colorRed" in header.point_fields
    hasNormals = False  # TODO

    # get number of scans
    imf = e57.image_file
    root = imf.root()
    scans = len(root["data3D"])

    # TODO: for scan_index in range(scans):
    # read scan at index 0
    scan_index = 0
    data = e57.read_scan(scan_index, colors=hasColor, ignore_missing_fields=True)

    points = np.vstack(
        (data["cartesianX"], data["cartesianY"], data["cartesianZ"])
    ).transpose()

    if not hasColor:
        return (points, np.full(points.shape, 1.0))

    colors = np.vstack(
        (data["colorRed"], data["colorGreen"], data["colorBlue"])
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


def read_e57(input: str, normals: bool = False) -> o3d.geometry.PointCloud:
    """Read a E57 file

    Parameters
    ----------
    input: str
        input file with path
    normals:
        Set to True to compute normals if missing. Default False

    Returns
    -------
    PointCloud
        Open3D point cloud format, optionally with normals
    """
    e57 = pye57.E57(input)
    points, colors = _get_points_colors(e57)

    return create_pcd(points, colors, normals)
