"Stitch 2 point clouds"
import math
import copy
from pathlib import Path
import open3d as o3d
from . import registration as reg
from . import noise_removal as nr

_DEBUG = False
_VERBOSE = False

VOXEL_SIZE = 0.0001

def color_obj(obj, color=(0,0,0)):
    "add color to object"
    obj.paint_uniform_color(color)
    return obj

def show_objects(obj, name=""):
    "Show the object list"
    o3d.visualization.draw_geometries(obj, window_name=name, width=1000, height=1000)

def decode_transformation(tm):
    "get information from translation matrix"     
    translation = tm[:3, 3]
    vx = tm[:3,0]
    vy = tm[:3,1]
    vz = tm[:3,2]
    if _DEBUG:
        print("Translation", translation)
        print(f"vx: {vx} vy: {vy} vz: {vz}")

    sx = math.sqrt(vx[0]**2+vx[1]**2+vx[2]**2)
    sy = math.sqrt(vy[0]**2+vy[1]**2+vy[2]**2)
    sz = math.sqrt(vz[0]**2+vz[1]**2+vz[2]**2)
    scale = [sx, sy, sz]
    if _DEBUG:
        print("Scale", scale)
    rot = [[tm[0,0]/sx,tm[0,1]/sy,tm[0,2]/sz,0],
           [tm[1,0]/sx,tm[1,1]/sy,tm[1,2]/sz,0],
           [tm[2,0]/sx,tm[2,1]/sy,tm[2,2]/sz,0],
           [0,0,0,1] ]
    if _DEBUG:
        print("rotation matrix", rot)
    rvx = math.atan2(rot[2][1],rot[2][2])
    rvy = math.atan2(-rot[2][0],math.sqrt(rot[2][1]**2+rot[2][2]**2))
    rvz = math.atan2(rot[1][0], rot[0][0])
    rotations = [rvx*180/math.pi, rvy*180/math.pi, rvz*180/math.pi]
    if _DEBUG:
        print ("Rotation angles", rotations)
    return translation, rotations, scale

def read_pointcloud(file: Path):
    "Read a standard pcl"
    if not file.exists():
        print("File does not exists")
        return None
    pcl = o3d.io.read_point_cloud(str(file))
    return pcl

def clean_point_cloud(pcd, epsilon=0.35, minimum_points=7, required_share =0.06):
    "clean pointcloud with Pre-stitching cleaning parameters"
    epsilon = 0.35
    minimum_points = 7
    required_share = 0.06
    epsilon = 0.001
    pcd_result, kept_indicies = nr.keep_significant_clusters(pcd, required_share, epsilon, minimum_points)
    if _DEBUG:
        print(f"Kept points: {len(kept_indicies)} Removing  {len(pcd.points) - len(kept_indicies)}")
    return pcd_result

def reg_point_clouds(ref, new):
    "register point cloud and find tranformatin bringing new to ref"
    test_target, transformation = reg.get_transformations(ref, new, VOXEL_SIZE)
    return test_target, transformation

def stitch_trans(reference, new, use_cleaning= False, use_color=False, debug=_DEBUG):
    "Get the best transformation for new"
    ref_pcl = copy.deepcopy(reference)
    new_pcl = copy.deepcopy(new)
    color=True
    if debug:
        print(f"Registation with color: {use_color}")
        print(f"Reference: {len(ref_pcl.points):8} Points, Color: {ref_pcl.has_colors()}")
        print(f"Test:      {len(new_pcl.points):8} Points, Color: {new_pcl.has_colors()}")
        color = True
    if color:
        ref_pcl.paint_uniform_color((0,1,0))
        new_pcl.paint_uniform_color((1,0,0))

    if debug:
        show_objects([ref_pcl, new_pcl], name="Original clouds")
    if use_cleaning:   # cleaning
        print("start cleaning")
        c_org = clean_point_cloud(ref_pcl)
        c_test = clean_point_cloud(new_pcl, epsilon=1)
        color_obj(c_test)
        objects = [c_org, c_test]
        show_objects(objects)

    test_target, transformation = reg_point_clouds(ref_pcl, new_pcl)
    if debug:
        print("Regisering test_target", test_target)
        print("Regisering transformation:", transformation)
    if debug:
        print("Transformation", transformation)
    return transformation
