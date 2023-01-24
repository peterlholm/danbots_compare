#!/bin/python3
"Compare a pointcloud to mesh"
import sys
from pathlib import Path
from random import random
import argparse
import open3d as o3d
import numpy as np

_DEBUG=False

def mesh_info(mesh):
    "print interesting info about mesh"
    print("Bounding box",mesh.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",mesh.get_oriented_bounding_box())
    print("Vertices", len(mesh.vertices))

def disturb_pcl(pcl_file, outfile, dist = 2):
    "create pcl with randome error dist"
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    olist = []
    for p in pcl.points:
        newp = (p[0]+ dist*random(), p[1]+ dist*random(), p[2]+ dist*random())
        olist.append(newp)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(olist)
    o3d.io.write_point_cloud(str(outfile), pcd)

def surface_to_pcl(mesh, alg="poisson", point_factor=10):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        mesh_info(mesh)
    no_points = len(mesh.vertices)//point_factor
    if _DEBUG:
        print("algorithm poisson", alg=="poisson")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=no_points)
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=no_points)
    #print("Resulting number of points", no_points)
    return pcl

def cmp_stl(mesh_file, pcl_file):
    "compare a mesh and a pointcloud and return a value for the error"
    if not mesh_file.exists() or not pcl_file.exists():
        print("File does not exist", mesh_file, pcl_file)
        raise Exception("Input not found")
    mesh = o3d.io.read_triangle_mesh(str(mesh_file))
    org = surface_to_pcl(mesh, point_factor=1)
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    if _DEBUG:
        #mesh_info(mesh)
        print("Points in reference", len(org.points))
        print("Points in pointcloud", len(pcl.points))
    dist = pcl.compute_point_cloud_distance(org)
    distance = np.asarray(dist)
    pclerror = np.sqrt(np.mean(distance ** 2))
    return pclerror

def cmp_pcl(org_file, pcl_file):
    "compare a org pcl and a pointcloud and return a value for the error"
    if not org_file.exists() or not pcl_file.exists():
        print("File does not exist", org_file, pcl_file)
        sys.exit(1)
    org = o3d.io.read_point_cloud(str(org_file))
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    if _DEBUG:
        print("Points in reference", len(org.points))
        print("Points in testfile", len(pcl.points))
    dist = pcl.compute_point_cloud_distance(org)
    distance = np.asarray(dist)
    pclerror = np.sqrt(np.mean(distance ** 2))
    return pclerror

if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='compare_pcl', description='Compare two pointclouds or a mesh and a pointcloud')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('file1')
    parser.add_argument('file2')
    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    fil1 = Path(args.file1)
    fil2 = Path(args.file2)
    if _DEBUG:
        print(f"Comparinge PCL {fil1} and {fil2}")
    if fil1.suffix=='.ply':
        error = cmp_pcl(fil1, fil2)
    elif fil1.suffix=='.stl':
        error = cmp_stl(fil1, fil2)
    else:
        print("Illegal file type")
        sys.exit(1)
    print(f"Stl: {fil1} pcl: {fil2} Error: {error:.2e}")


if __name__=="__mainx__":
    BASEDIR = Path(__file__).parent / 'testdata'
    disturb_pcl(BASEDIR / 'LJ3.ply', BASEDIR / 'LJ3_disturb.ply')
    error = cmp_pcl(BASEDIR / 'LJ3.ply', BASEDIR / 'LJ3.ply')
    print(f"Error: {error:.2e}")
    error = cmp_pcl(BASEDIR / 'LJ3.ply', BASEDIR / 'LJ3_disturb.ply')
    print(f"Error: {error:.2e}")
    files = [('LJ3.stl', 'LJ3.ply'),('LJ3.stl', 'LJ3_face.ply'),('LJ3.stl', 'LJ3_face2.ply'),('LJ3.stl', 'LJ3_dist.ply')]
    # files = [('t9UJscan.stl', 'LJ3_face2.ply')]
    # files = [('LJ3.stl', 'LJ3.ply')]
    files = []
    for i,o in files:
        inmesh = BASEDIR / "testdata" / i
        inpcl = BASEDIR / "testdata" / o
        error = cmp_stl(inmesh, inpcl)
        print(f"Stl: {i} pcl: {o} Error: {error:.2e}")
