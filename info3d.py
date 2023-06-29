#!/usr/bin/python3
"Info on 3d object"
import sys
from pathlib import Path
import argparse
import open3d as o3d


def obj_size(mesh : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = mesh.get_max_bound()
    min_size = mesh.get_min_bound()
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s

def obj_info(obj):
    "print interesting info about mesh"
    print("Bounding box",obj.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",obj.get_oriented_bounding_box())

    if isinstance(obj, o3d.geometry.PointCloud):
        if obj.has_points():
            print(f"Pointcloud with {len(obj.points)} Points")
        if obj.has_colors():
            print("Object has colors")
        else:
            print("Object has no colors")
        if obj.has_normals():
            print("Object has normals")

    if isinstance(obj, o3d.geometry.TriangleMesh):
        print("Vertices", len(obj.vertices))
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

if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='info3d', description='Show information on 3d object file')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('org_file', help="The stl or pointcloud file")

    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _VERBOSE = args.v
    fil1 = Path(args.org_file)
    if _VERBOSE:
        print(f"Information on {fil1}")
    # check files exists
    if not fil1.exists():
        print("input file does not exist")
        sys.exit(1)
    # check file types

    if fil1.suffix=='.stl':
        inobj = o3d.io.read_triangle_mesh(str(fil1))
    elif fil1.suffix=='.ply':
        inobj = o3d.io.read_point_cloud(str(fil1))
    else:
        print("Input file type error")
        sys.exit(1)

    obj_size = obj_size(inobj)
    print(f"Input object size {obj_size:.3f}")

    obj_info(inobj)
    