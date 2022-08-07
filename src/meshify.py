import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from common.las import read_las


def _postprocess_mesh(mesh, max_triangles: int = None, clean: bool = True):

    if max_triangles is not None:
        mesh = mesh.simplify_quadric_decimation(max_triangles)

    if clean:
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()

    return mesh


def _export_mesh(mesh, path: str):
    if not path.endswith(".ply"):
        raise ValueError("file name must have '.ply' extension")
    o3d.io.write_triangle_mesh(path, mesh)


def meshify_bpa(
    pcd: o3d.geometry.PointCloud,
    output: str,
    max_triangles: int = None,
    clean: bool = True,
):
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist

    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector([radius, radius * 2])
    )

    # downsample the mesh to an acceptable number of triangles
    dec_mesh = _postprocess_mesh(bpa_mesh, max_triangles, clean)

    _export_mesh(dec_mesh, output)


def meshify_poisson(
    pcd: o3d.geometry.PointCloud,
    output: str,
    depth: int = 8,
    scale: float = 1.1,
    max_triangles: int = None,
    clean: bool = True,
):
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, width=0, scale=scale, linear_fit=False
    )[0]

    # To get a clean result, it is often necessary to add a cropping step to clean
    # unwanted artifacts highlighted as yellow from the left image below
    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)

    # downsample the mesh to an acceptable number of triangles
    dec_mesh = _postprocess_mesh(p_mesh_crop, max_triangles, clean)

    _export_mesh(dec_mesh, output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--input", type=str, required=True)

    args = parser.parse_args()

    pcd = read_las(args.input, normals=True)

    base_name = Path(args.input).name[:-4]
    output_dir = f"{args.input[:-4]}"
    Path(output_dir).mkdir(exist_ok=True)

    meshify_bpa(pcd, f"{output_dir}/{base_name}_bpa.ply")
    meshify_poisson(pcd, f"{output_dir}/{base_name}_poisson.ply", depth=8)
