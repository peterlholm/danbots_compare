#!/bin/python3
"Convert a 3d file to pointcloud"
import sys
import math
import copy
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

def rotate(in_pcl, rotation_angles):
    "rotate input pcl rotation angles in degree"
    rotation_radians = [rotation_angles[0] / 180 * math.pi, rotation_angles[1] / 180 * math.pi, rotation_angles[2] /180 * math.pi]
    r_matrix = o3d.geometry.PointCloud.get_rotation_matrix_from_axis_angle(rotation_radians)
    opcl = in_pcl.rotate(r_matrix, center=(0,0,0))
    return opcl

def scale(in_pcl, scalefactor):
    "scale object around 0.0"
    opcl = in_pcl.scale(scalefactor, center=[0,0,0])
    return opcl

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Transform 3d pcl files ')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-s', required=False, help="Show output result", action='store_true' )
    parser.add_argument('-a', "--axis", required=False, help="Show axis", action='store_true' )
    parser.add_argument('org_file', type=Path, help="The original pointcloud")
    parser.add_argument('out_file', type=Path, help="The resulting pointcloud")
    parser.add_argument('-c', "--center", required=False, help="Center the object", action='store_true' )
    parser.add_argument('-tx', required=False, default=0, type=float, help="Transform in x direction", action='store' )
    parser.add_argument('-ty', required=False, default=0, type=float, help="Transform in y direction", action='store' )
    parser.add_argument('-tz', required=False, default=0, type=float, help="Transform in z direction", action='store' )
    parser.add_argument('-rx', required=False, default=0, type=float, help="Rotate around x axis", action='store' )
    parser.add_argument('-ry', required=False, default=0, type=float, help="Rotate around y axis", action='store' )
    parser.add_argument('-rz', required=False, default=0, type=float, help="Rotate around z axis", action='store' )
    parser.add_argument('--scale', required=False, default=0, type=float, help="Scale object", action='store' )
    args = parser.parse_args()

    _VERBOSE = args.v
    _DEBUG=args.d
    if _DEBUG:
        print(args)
    if _VERBOSE:
        print(f"Converting {args.org_file} to {args.out_file}")
    # check files exists
    if not args.org_file.exists():
        print("input file does not exist")
        sys.exit(1)
    if args.org_file.suffix=='.ply':
        # pcl imput
        inpcl = o3d.io.read_point_cloud(str(args.org_file))
        if _VERBOSE:
            obj_size = obj_size(inpcl)
            print(f"Input object size {obj_size:.2f} m")
    elif args.org_file.suffix=='.stl':
        inpcl = o3d.io.read_triangle_mesh(str(args.org_file))
    else:
        print("Input file type error")
        sys.exit(1)
    # check output file types
    if args.out_file.suffix in ['.ply', '.stl']:
        if not Path(args.out_file).parent.exists():
            print(f"output folder {args.out_file} does not exisit")
            sys.exit(2)

    if _DEBUG:
        obj_info(inpcl)
    if args.s:
        org = copy.deepcopy(inpcl)

    # if _VERBOSE:
    #     print(f"Transforming ({args.x},{args.y},{args.z}")
    if args.center:
        center = inpcl.get_center()
        if _DEBUG:
            print(f"Center: {center}")
        outpcl = inpcl.translate(-center)

    if args.scale:
        if _VERBOSE:
            print("Performing scaling", (args.scale))
        outpcl = scale(inpcl,(args.scale))

    if args.tx or args.ty or args.tz:
        if _VERBOSE:
            print("performing translation ",(args.tx, args.ty, args.tz))
        outpcl = trans(inpcl, (args.tx, args.ty, args.tz))
    if args.rx or args.ry or args.rz:
        if _VERBOSE:
            print("Performing rotation", (args.rx,args.ry,args.rz))
        outpcl = rotate(inpcl,(args.rx,args.ry,args.rz))

    if isinstance(outpcl, o3d.geometry.PointCloud):
        o3d.io.write_point_cloud(str(args.out_file), outpcl)
    elif isinstance(outpcl, o3d.geometry.TriangleMesh):
        outpcl.compute_triangle_normals()
        o3d.io.write_triangle_mesh(str(args.out_file), outpcl)
    else:
        print("illegal output")

    if args.s:
        if isinstance(outpcl, o3d.geometry.PointCloud):
            if not outpcl.has_normals():
                outpcl.compute_normals()
        elif isinstance(outpcl, o3d.geometry.TriangleMesh):
            if not outpcl.has_triangle_normals():
                outpcl.compute_triangle_normals()
        else:
            print("wrong object type")

        objects = [org, outpcl]
        if args.axis:
            axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0,0,0])
            objects.append(axis)
        o3d.visualization.draw_geometries(objects, width=1000, height=1000)
