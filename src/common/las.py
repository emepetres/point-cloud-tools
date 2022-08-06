from typing import Tuple

import laspy as lp
import numpy as np


def read_las(input: str) -> Tuple[np.ndarray, np.ndarray]:
    """Read a LAS/LAZ file

    Parameters
    ----------
    input: str
        input file with path

    Returns
    -------
    tuple
        xyz points and rgb colors numpy arrays in (3, n) shape
    """
    point_cloud = lp.read(input)

    # store coordinates in "points", and colors in "colors" variable
    points = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z))
    colors = np.vstack(
        (point_cloud.red, point_cloud.green, point_cloud.blue)
    )

    return (points, colors)
