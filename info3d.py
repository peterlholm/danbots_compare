#!/usr/bin/python3
"Info on 3d object"

#   230501  PLH     First version
#   230817  PLH     STL info updated
import sys
from pathlib import Path
import math
import argparse
#import numpy as np
import open3d as o3d


def obj_size(mesh : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = mesh.get_max_bound()
    min_size = mesh.get_min_bound()
    #diameter = np.linalg.norm(np.asarray(max_size) - np.asarray(min_size))
    #print("diameter", diameter)
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s

def obj_info(obj):
    "print interesting info about mesh"
    al_box = obj.get_axis_aligned_bounding_box()
    max_size = obj.get_max_bound()
    min_size = obj.get_min_bound()
    distx = max_size[0]-min_size[0]
    disty = max_size[1]-min_size[1]
    distz = max_size[2]-min_size[2]
    vol  = distx * disty * distz
    print(al_box)
    print(obj.get_oriented_bounding_box())

    newlist = sorted([distx, disty, distz])
    area = newlist[1] * newlist[2]
    print(f"SurfaceArea:   {area:.5f}")
    if isinstance(obj, o3d.geometry.PointCloud):
        print(f"Point:         {len(obj.points)}")
        print(f"Colors:        {obj.has_colors()}")
        print(f"Normals:       {obj.has_normals()}")
        print(f"Center:        {obj.get_center()}")
        resolution = len(obj.points) / area
        print(f"Est. Resol:    {resolution:.4} point / unit*2    ({(1/math.sqrt(resolution)):.3f} unit/point)")


    if isinstance(obj, o3d.geometry.TriangleMesh):
        print(f"Vertices:      {obj.has_vertices()}")   # hjørner
        print(f"Vertices:      {len(obj.vertices)}")
        print(f"Vertex Colors: {obj.has_vertex_colors()}")
        print(f"Vertex Normals:{obj.has_vertex_normals()}")
        if obj.has_triangles():
            print(f"Triangles:     {obj.has_triangles()}")
            print(f"Triangles #:   {len(obj.triangles)}")
            print(f"Triang.Normals:{obj.has_triangle_normals()}")
        print(f"TriMaterialID: {obj.has_triangle_material_ids()}")
        #print(obj.triangle_material_ids)
        print(f"Textures:      {obj.has_textures()}")
        if obj.has_textures():
            print(f"Textures:      {len(obj.textures)}")
            #img = obj.textures[0]
            #o3d.io.write_image("image.png", img)
            #print(f"Type of textures {type(obj.textures[0])}")
        print(f"Center:        {obj.get_center()}")
        print(f"SurfaceArea:   {obj.get_surface_area():.5f}")
        if obj.is_watertight():
            print(f"Volume     :   {obj.get_volume()}")
        resolution = len(obj.vertices) / obj.get_surface_area()
        print(f"Area Resol.:   {resolution:.4} point / unit*2")
 
    print("-----------------------------------------------------------")
    print(f"Size:          {distx:.3f} x {disty:.3f} x {distz:.3f}")
    print(f"Vol:           {vol:.5f}")
    print(f"BoxArea:       {area:.5f}")

    if obj_size(obj) > 2:
        unit = "mm"
    else:
        unit = "m"
    print(f"Unit:          {unit}")
    print(f"Point Dist:    {math.sqrt(resolution):.4f} point / unit ({(1/math.sqrt(resolution)):.3f} unit/point)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='info3d', description='Show information on 3d object files in .stl and .ply')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('org_file', help="The stl or pointcloud file")

    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _VERBOSE = args.v
    fil1 = Path(args.org_file)
    if _VERBOSE:
        print(f"Information from: {fil1}")
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

    obj_info(inobj)
    