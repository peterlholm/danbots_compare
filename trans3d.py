#!/bin/python3
"Convert a 3d file to pointcloud"
import sys
from pathlib import Path
import argparse
import open3d as o3d

_DEBUG = False
_VERBOSE = False

def obj_info(obj):
    "print interesting info about mesh"
    print("Bounding box",obj.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",obj.get_oriented_bounding_box())

    if isinstance(obj, o3d.geometry.PointCloud):
        print("Pointcloud")
        if obj.has_points():
            print(f"Obj with {len(obj.points)} Points")
        if obj.has_colors():
            print("Object has colors")
        else:
            print("Object has no colors")
        if obj.has_normals():
            print("Object has normals")
    # mesh
    if isinstance(obj, o3d.geometry.TriangleMesh):
        print("TriangleMesh\nVertices", len(obj.vertices))
        if obj.has_vertex_colors():
            print("Object has colors")
        else:
            print("Object has no colors")
        if obj.has_vertex_normals():
            print("Object has vertex normals")
        if obj.has_vertices():
            print("Object has vertices")
        if obj.has_triangle_normals():
            print("Object has triangle normals")

def stl2pcl(mesh):
    "create pointcloud from vertices in stl"
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    return pcd

def obj_size(mesh : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = mesh.get_max_bound()
    min_size = mesh.get_min_bound()
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s

def trans(in_pcl, translation):
    "transform input pcl to output pcl"
    opcl = in_pcl.translate(translation)
    return opcl

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Transform 3d pcl files ')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('org_file', type=Path, help="The original pointcloud")
    parser.add_argument('out_file', type=Path, help="The resulting pointcloud")
    parser.add_argument('-x', required=False, default=0, type=float, help="Transform in x direction", action='store' )
    parser.add_argument('-y', required=False, default=0, type=float, help="Transform in y direction", action='store' )
    parser.add_argument('-z', required=False, default=0, type=float, help="Transform in z direction", action='store' )
    args = parser.parse_args()

    _VERBOSE = args.v
    _DEBUG=args.d
    if _DEBUG:
        print(args)
    if _VERBOSE:
        print(f"Converting {args.org_file} to {args.out_file} Transforming ({args.x},{args.y},{args.z}")
    # check files exists
    if not args.org_file.exists():
        print("input file does not exist")
        sys.exit(1)
    # check file types
    if args.out_file.suffix != '.ply':
        print("only convertion to pointcloud supported")
        sys.exit(2)
    if args.org_file.suffix=='.ply':
        # pcl imput
        inpcl = o3d.io.read_point_cloud(str(args.org_file))
        if _VERBOSE:
            obj_size = obj_size(inpcl)
            print(f"Input object size {obj_size:.2f} m")
    else:
        print("Input file type error")
        sys.exit(1)
    if _DEBUG:
        obj_info(inpcl)

    # if _VERBOSE:
    #     print(f"Transforming ({args.x},{args.y},{args.z}")

    outpcl = trans(inpcl, (args.x, args.y, args.z))
    #all convertions done - write file

    o3d.io.write_point_cloud(str(args.out_file), outpcl)
