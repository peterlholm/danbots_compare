#!/bin/python3
"Stitch a pointcloud to pointcloud"
import sys
from pathlib import Path
import argparse
import open3d as o3d
#import numpy as np
from stitching.stitch import stitch_trans, decode_transformation


_DEBUG = False
_SHOW = True
_VERBOSE = False


def surface_to_pcl(mesh, alg="poisson", number_of_points=100000, init_factor=10):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        mesh_info(mesh)
        print(f"Algoritm: {alg} Number_of_points: {number_of_points} Point_factor: {init_factor}")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=number_of_points, )
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=number_of_points, init_factor=init_factor)
    return pcl

def stl2pcl(mesh):
    "create pointcloud from vertices in stl"
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    return pcd

def mesh_size(mesh : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = mesh.get_max_bound()
    min_size = mesh.get_min_bound()
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s

def mesh_info(mesh):
    "print interesting info about mesh"
    print("Bounding box",mesh.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",mesh.get_oriented_bounding_box())
    print("Vertices", len(mesh.vertices))

# def draw_registration_result(reference, test_source, transformation, axis=False, window_name="registration result", color=False):
#     "Debug draw registration result"
#     reference_temp = copy.deepcopy(reference)
#     test_temp = copy.deepcopy(test_source)
#     if color:
#         reference_temp.paint_uniform_color([0, 0.7, 0.1])
#         test_temp.paint_uniform_color([1, 0.0, 0.1])
#     test_temp.transform(transformation)
#     pointclouds =[reference_temp, test_temp]
#     if axis:
#         axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
#         pointclouds.append(axis_pcd)
#     o3d.visualization.draw_geometries(pointclouds, window_name=window_name)

def trans_file(pcl, outfile, transformation):
    "transform and write file with transformation"
    #in_pcl = o3d.io.read_point_cloud(str(infile))
    pcl.transform(transformation)
    o3d.io.write_point_cloud(str(outfile), pcl)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='stitch3d', description='Stitch two 3d files and calculate error figures')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('org_file', type=Path, help="The original stl or pointcloud")
    parser.add_argument('test_file', type=Path, help="The pointcloud to be measured")
    parser.add_argument('-o', required=False, help="Make an output file", action='store_true' )
    args = parser.parse_args()

    _DEBUG = args.d
    _VERBOSE = args.v
    if _VERBOSE:
        print(f"Stitching {str(args.org_file)} and {args.test_file}")
    # check files exists
    if not args.org_file.exists() or not args.test_file.exists():
        print("input file(s) does not exist")
        sys.exit(1)
    # check file types

    if args.org_file.suffix=='.stl':
        inmesh = o3d.io.read_triangle_mesh(str(args.org_file))
        obj_size = mesh_size(inmesh)
        if _VERBOSE:
            print(f"Input object size {obj_size:.2f}")
        in_pcl = surface_to_pcl(inmesh)
    elif args.org_file.suffix=='.ply':
        in_pcl = o3d.io.read_point_cloud(str(args.org_file))
    else:
        print("Input file type error")
        sys.exit(1)

    if args.test_file.suffix=='.ply':
        t_pcl = o3d.io.read_point_cloud(str(args.test_file))
    else:
        print("Input file2 type error")
        sys.exit(1)

    transform = stitch_trans(in_pcl, t_pcl, debug=args.d, verbose=args.v)
    print("Resulting Transformation:", transform)
    trans, rot, scale = decode_transformation(transform)
    print(f"Translation, {trans}, Rotation {rot}, scale {scale}", )
 
    if transform is None:
        print("No transformation can be found")
        sys.exit(2)
    if args.o:
        trans_file(t_pcl, "out.ply", transform)
