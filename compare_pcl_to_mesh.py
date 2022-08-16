"Compare a pointcloud to mesh"
from pathlib import Path
import open3d as o3d
import numpy as np

_DEBUG=False

def mesh_info(mesh):
    "print interesting info about mesh"
    print("Bounding box",mesh.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",mesh.get_oriented_bounding_box())
    print("Vertices", len(mesh.vertices))

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

def cmp(mesh_file, pcl_file):
    "compare a mesh and a pointcloud and return a value for the error"
    if not mesh_file.exists() or not pcl_file.exists():
        print("File does not exist", mesh_file, pcl_file)
        raise Exception("Input not found")
    mesh = o3d.io.read_triangle_mesh(str(mesh_file))
    print(mesh)
    org = surface_to_pcl(mesh, point_factor=1)
    pcl = o3d.io.read_point_cloud(str(pcl_file))
    if _DEBUG:
        #mesh_info(mesh)
        print("Points in original", len(org.points))
        print("Point in pointcloud", len(pcl.points))
    dist = pcl.compute_point_cloud_distance(org)
    distance = np.asarray(dist)
    pclerror = np.sqrt(np.mean(distance ** 2))
    return pclerror

if __name__=="__main__":
    files = [('LJ3.stl', 'LJ3.ply'),('LJ3.stl', 'LJ3_face.ply'),('LJ3.stl', 'LJ3_face2.ply'),('t9UJscan.stl', 'LJ3_face2.ply')]
    files = [('t9UJscan.stl', 'LJ3_face2.ply')]
    BASEDIR = Path(__file__).parent
    for i,o in files:
        inmesh = BASEDIR / "testdata" / i
        inpcl = BASEDIR / "testdata" / o
        error = cmp(inmesh, inpcl)
        print(f"Stl: {i} pcl: {o} Error: {error:.2e}")
