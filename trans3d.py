#!/bin/python3
"Convert a 3d file to pointcloud"
import sys
from pathlib import Path
from random import random
import argparse
import open3d as o3d

_DEBUG = False
_VERBOSE = True


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

def mesh_info(mesh):
    "print interesting info about mesh"
    print("Bounding box",mesh.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",mesh.get_oriented_bounding_box())
    print("Vertices", len(mesh.vertices))

def disturb_pcl(ipcl, dist=2):
    "create pcl with randome error dist"
    def random_error(dist):
        return 2 * random()*dist - dist/2

    olist = []
    for p in ipcl.points:
        newp = (p[0] + random_error(dist), p[1] + random_error(dist), p[2] + random_error(dist))
        #newp = (p[0], p[1]+ dist, p[2])
        olist.append(newp)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(olist)
    if ipcl.has_colors():
        pcd.colors = o3d.utility.Vector3dVector(ipcl.colors)
    return pcd

def surface_to_pcl(mesh, alg="poisson", point_factor=10, points=None):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        mesh_info(mesh)
    if not points is None:
        no_points = points
    else:
        no_points = len(mesh.vertices)//point_factor
    if _DEBUG:
        print("ALGOrithm poisson", alg=="poisson")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=no_points)
    elif alg=='uniformly':
        pcl = mesh.sample_points_uniformly(number_of_points=no_points)
    else:
        print("unknown sampling")
        sys.exit(2)
    #print("Resulting number of points", no_points)
    return pcl

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Transform 3d pcl files ')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('org_file', help="The original pointcloud")
    parser.add_argument('out_file', help="The resulting pointcloud")
    parser.add_argument('-x', required=False, default=None, type=float, help="Transfor in x direction", action='store' )
    args = parser.parse_args()
    _VERBOSE = args.v
    if args.d:
        _DEBUG=True
    if _DEBUG:
        _VERBOSE=True

    fil1 = Path(args.org_file)
    fil2 = Path(args.out_file)
 
    if _VERBOSE:
        print(f"Converting {fil1} to {fil2}")

    # check files exists
    if not fil1.exists():
        print("input file(s) does not exist")
        sys.exit(1)

    # check file types
    if fil2.suffix != '.ply':
        print("only convertion to pointcloud supported")
        sys.exit(2)

    if fil1.suffix=='.ply':
        # pcl imput
        inpcl = o3d.io.read_point_cloud(str(fil1))
        if _VERBOSE:
            obj_size = obj_size(inpcl)
            print(f"Input object size {obj_size:.2f} m")
    else:
        print("Input file type error")
        sys.exit(1)
    if _VERBOSE:
        obj_info(inpcl)
    #all convertions done - write file

    o3d.io.write_point_cloud(str(fil2), inpcl)
