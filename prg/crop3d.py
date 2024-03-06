#!/usr/bin/python3
"Info on 3d object"
import sys
import copy
from pathlib import Path
import argparse
import open3d as o3d

_DEBUG = False
_VERBOSE = False

def show_objects(objlist, name="", axis=True):
    "Show the object list"
    if axis:
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=(0,0,0))
        objlist.append(coord)
    o3d.visualization.draw_geometries(objlist, window_name=name, width=1000, height=1000)

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
    parser = argparse.ArgumentParser(description='Crop the model to the given size', epilog="See also info3d, show3d, trans3d, conv3d, mirror3d, stitch3d")
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-s', "--show", required=False, help="Show output", action='store_true' )
    parser.add_argument('-c', "--center", required=False, help="Center the object", action='store_true' )
    parser.add_argument('-xh', required= False, type=float, help="maximum X value")
    parser.add_argument('-xl', required= False, type=float, help="minimum X value")
    parser.add_argument('-yh', required= False, type=float, help="maximum Y value")
    parser.add_argument('-yl', required= False, type=float, help="minimum Y value")
    parser.add_argument('-zh', required= False, type=float, help="maximum Z value")
    parser.add_argument('-zl', required= False, type=float, help="minimum Z value")
    parser.add_argument('-o', '--output', required=False, type=Path, help="Output file", metavar="Outputfile")
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
    if _VERBOSE:
        obj_size = obj_size(inobj)
        print(f"Input object size {obj_size:.2f}")
    bounding_box = inobj.get_axis_aligned_bounding_box()
    b_min = copy.deepcopy(bounding_box.min_bound)
    b_max = copy.deepcopy(bounding_box.max_bound)
    if args.xl:
        b_min[0] = args.xl
    if args.xh:
        b_max[0] = args.xh
    if args.yl:
        b_min[1] = args.yl
    if args.yh:
        b_max[1] = args.yh
    if args.zl:
        b_min[2] = args.zl
    if args.zh:
        b_max[2] = args.zh

    bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=b_min, max_bound=b_max)
    out_obj = inobj.crop(bounding_box)

    if args.center:
        center = out_obj.get_center()
        if _DEBUG:
            print(f"Center: {center}")
        out_obj.translate(-center)
    if args.show:
        show_objects([out_obj], name=fil1.name)
    if args.output:
        print(args.output)
        o3d.io.write_point_cloud(str(args.output), out_obj)
