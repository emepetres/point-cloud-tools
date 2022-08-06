import argparse
from typing import Tuple

import numpy as np
import open3d as o3d

from common.las import read_las


def _create_pcd(points: np.array, colors: np.array) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.transpose())
    pcd.colors = o3d.utility.Vector3dVector(colors.transpose() / 255)
    # # pcd.normals = o3d.utility.Vector3dVector(point_cloud[:,6:9])

    return pcd


def _postprocess_mesh(mesh, triangles: int = 10000, clean: bool = False):

    dec_mesh = mesh.simplify_quadric_decimation(triangles)

    if clean:
        dec_mesh.remove_degenerate_triangles()
        dec_mesh.remove_duplicated_triangles()
        dec_mesh.remove_duplicated_vertices()
        dec_mesh.remove_non_manifold_edges()


def _export_mesh(mesh, path: str):
    if not path.endswith(".ply"):
        raise ValueError("file name must have '.ply' extension")
    o3d.io.write_triangle_mesh(path, mesh)


def meshify_bpa(
    points: np.array,
    colors: np.array,
    output: str,
    depth: int = 8,
    scale: Tuple[float, float] = (1, 1),
    triangles: int = 10000,
    clean: bool = False,
):
    pcd = _create_pcd(points, colors)

    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist

    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector([radius, radius * 2])
    )

    # downsample the mesh to an acceptable number of triangles
    dec_mesh = _postprocess_mesh(bpa_mesh, triangles, clean)

    _export_mesh(dec_mesh, output)


def meshify_poisson(
    points: np.array,
    colors: np.array,
    output: str,
    depth: int = 8,
    scale: float = 1.1,
    triangles: int = 10000,
    clean: bool = False,
):
    pcd = _create_pcd(points, colors)

    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, width=0, scale=scale, linear_fit=False
    )[0]

    # To get a clean result, it is often necessary to add a cropping step to clean
    # unwanted artifacts highlighted as yellow from the left image below
    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)

    # downsample the mesh to an acceptable number of triangles
    dec_mesh = _postprocess_mesh(p_mesh_crop, triangles, clean)

    _export_mesh(dec_mesh, output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--input", type=str, required=True)

    args = parser.parse_args()

    points, colors = read_las(args.input)

    # meshify_bpa(points, colors, f"{args.input[:-4]}_bpa.ply")
    meshify_poisson(points, colors, f"{args.input[:-4]}_poisson.ply")
