import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from formats.las import read_las
from formats.e57 import read_e57


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
    radius_scale: 3,
    max_triangles: int = None,
    clean: bool = True,
):
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = radius_scale * avg_dist

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
    parser = argparse.ArgumentParser(
        description="Transform a point cloud into a triangle mesh"
    )

    parser.add_argument(
        "--input", type=str, required=True, help="Path to point cloud file"
    )
    parser.add_argument(
        "-bpa", action="store_true", default=False, help="Use BPA strategy"
    )
    parser.add_argument(
        "-poisson", action="store_true", default=False, help="Use Poisson strategy"
    )
    parser.add_argument(
        "--radius", type=int, default=3, help="Scale of the radius (only for BPA)"
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=8,
        help="Increase the depth to get more details (only for Poisson)",
    )
    parser.add_argument(
        "--max_triangles",
        type=int,
        default=None,
        help="Reduce the output mesh to the specified triangles",
    )

    args = parser.parse_args()

    ext = args.input[-4:]
    if ext == ".las" or ext == ".laz":
        pcd = read_las(args.input, normals=True)
    elif ext == ".e57":
        pcd = read_e57(args.input, normals=True)
    else:
        raise argparse.ArgumentError("Input file type not supported.")

    if not args.bpa and not args.poisson:
        raise argparse.ArgumentError(
            "At least BPA or Poisson strategy must be selected"
        )

    base_name = Path(args.input).name[:-4]
    output_dir = f"{args.input[:-4]}"
    Path(output_dir).mkdir(exist_ok=True)

    # config suffix
    bpa_suffix = "_bpa"
    pss_suffix = "_poisson"
    if args.radius != 3:
        bpa_suffix += f"_r{args.radius}"
    if args.depth != 8:
        pss_suffix += f"_d{args.depth}"
    if args.max_triangles is not None:
        bpa_suffix += f"_{args.max_triangles}"
        pss_suffix += f"_{args.max_triangles}"

    if args.bpa:
        meshify_bpa(
            pcd,
            f"{output_dir}/{base_name}{bpa_suffix}.ply",
            radius_scale=args.radius,
            max_triangles=args.max_triangles,
        )
    if args.poisson:
        meshify_poisson(
            pcd,
            f"{output_dir}/{base_name}{pss_suffix}.ply",
            depth=args.depth,
            max_triangles=args.max_triangles,
        )
