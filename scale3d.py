#!/bin/python3
"Compare a pointcloud to mesh"
import sys
from pathlib import Path
from random import random
import argparse
import open3d as o3d
import numpy as np


def obj_size(mesh : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = mesh.get_max_bound()
    min_size = mesh.get_min_bound()
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s



def scale_obj(obj, factor):
    obj.scale(factor, (0,0,0))

    return obj


if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='scale3d', description='Scale a 3d files with factor')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-s', required=False, type=float, default=0.01, help="Give scale factor", action='store')
    parser.add_argument('in_file', help="The original stl or pointcloud")
    parser.add_argument('out_file', help="The output file")
    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _VERBOSE = args.v
    fil1 = Path(args.in_file)
    fil2 = Path(args.out_file)
    scale = args.s
    #print ("s", args.s)
    if _VERBOSE:
        print(f"Scaling {fil1} with {scale}")
    # check files exists
    if not fil1.exists():
        print("input file(s) does not exist")
        sys.exit(1)
    # check file types
    if fil2.exists():
        print("output file allready exist")
        sys.exit(1)
    # check file types

    if fil1.suffix=='.stl':
        inobj = o3d.io.read_triangle_mesh(str(fil1))
    elif fil1.suffix=='.ply':
        inobj = o3d.io.read_point_cloud(str(fil1))
    else:
        print("Input file type error")
        sys.exit(1)

    o_size = obj_size(inobj)
    if _VERBOSE:
        print(f"Input object size {o_size:.2f}")

    obj1 = scale_obj(inobj, scale)

    if isinstance(obj1, o3d.geometry.TriangleMesh):
        obj1.compute_triangle_normals()
        o3d.io.write_triangle_mesh(str(fil2), obj1)
    else:
        o3d.io.write_point_cloud(str(fil2), obj1)
    if _VERBOSE:
        o_size = obj_size(obj1)
        print(f"Output object size {o_size:.3f}")

