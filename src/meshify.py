import argparse

import numpy as np
import open3d as o3d

from common.las import read_las


def 


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--input", type=str, required=True)

    args = parser.parse_args()

    points, colors = read_las(args.input)
