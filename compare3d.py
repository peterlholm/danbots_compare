#!/bin/python3
"Compare a pointcloud to mesh"
import sys
from pathlib import Path
import argparse
import open3d as o3d
import numpy as np

_DEBUG = False
_SHOW = True
_VERBOSE = True

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

def cmp_stl(mesh_file, pcl_file):
    "compare a mesh and a pointcloud and return a value for the error"
    if not mesh_file.exists() or not pcl_file.exists():
        print("File does not exist", mesh_file, pcl_file)
        raise FileNotFoundError("Input not found")
    mesh = o3d.io.read_triangle_mesh(str(mesh_file))
    org = stl2pcl(mesh)
    #org = surface_to_pcl(mesh, point_factor=1)
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    if _DEBUG:
        #mesh_info(mesh)
        print("Points in reference", len(org.points))
        print("Points in pointcloud", len(pcl.points))
    if _SHOW:
        org.paint_uniform_color([0,0,1])
        pcl.paint_uniform_color([0.5,0.5,0])
        o3d.visualization.draw_geometries([org,pcl])
    dist = pcl.compute_point_cloud_distance(org)
    print(dist)
    distance = np.asarray(dist)
    print(distance)
    pclerror = np.sqrt(np.mean(distance ** 2))
    if _DEBUG:
        print("Min:", np.min(distance))
        print("Max:", np.max(distance))
        print("Mean:", np.mean(distance))
        print("MSQ:", pclerror)
    return pclerror, np.min(distance), np.max(distance), np.mean(distance)

def cmp2pcl(org_pcl, test_pcl):
    "compare 2 pcl a pointcloud and return a value for the error"
    if _VERBOSE:
        print("No Points in reference", len(org_pcl.points))
        print("No Points in testfile", len(test_pcl.points))
    dist = test_pcl.compute_point_cloud_distance(org_pcl)
    #print(dist)
    distance = np.asarray(dist)
    if _DEBUG:
        print(distance)
    pclerror = np.sqrt(np.mean(distance ** 2))
    if _DEBUG:
        print(f"Min error:  {np.min(distance):.6f} m")
        print(f"Max error:  {np.max(distance):.6f} m")
        print(f"Mean error: {np.mean(distance):.6f} m")
        print(f"RMS:        {pclerror:.6f} m")
    return pclerror, np.min(distance), np.max(distance), np.mean(distance)

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
    parser = argparse.ArgumentParser(prog='compare3d', description='Compare two 3d files and calculate error figures')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('org_file', help="The original stl or pointcloud")
    parser.add_argument('test_file', help="The pointcloud to be measured")
    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _VERBOSE = args.v
    fil1 = Path(args.org_file)
    fil2 = Path(args.test_file)
    if _VERBOSE:
        print(f"Comparing {fil1} and {fil2}")
    # check files exists
    if not fil1.exists() or not fil2.exists():
        print("input file(s) does not exist")
        sys.exit(1)
    # check file types

    if fil1.suffix=='.stl':
        inmesh = o3d.io.read_triangle_mesh(str(fil1))
        obj_size = mesh_size(inmesh)
        if _VERBOSE:
            print(f"Input object size {obj_size:.2f} m")
        in_pcl = surface_to_pcl(inmesh)
    elif fil1.suffix=='.ply':
        in_pcl = o3d.io.read_point_cloud(str(fil1))
        obj_size = mesh_size(in_pcl)
        if _VERBOSE:
            print(f"Input object size {obj_size:.2f} m")
    else:
        print("Input file type error")
        sys.exit(1)

    if fil2.suffix=='.ply':
        t_pcl = o3d.io.read_point_cloud(str(fil2))

    if _VERBOSE:
        print(f"input pointcloud with {len(in_pcl.points)}")

    rms, vmin, vmax, mean = cmp2pcl(in_pcl, t_pcl)
    print(f"Point ABS distance Error: RMS: {rms:.6f} m Min: {vmin:.6f} m Max: {vmax:.6f} m Mean: {mean:.6f} m")
    print(f"Point Rel distance Error: RMS: {rms/obj_size*100:.3f}% Min: {vmin/obj_size*100:.3f}% Max: {vmax/obj_size*100:.3f}% Mean: {mean/obj_size*100:.3f}%")
