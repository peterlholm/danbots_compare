#!/usr/bin/python3
"Show 1/2 3d pictures"
import sys
from pathlib import Path
import argparse
import open3d as o3d

_DEBUG = True
_VERBOSE=True
ZOOM = 0.6
CAM_POSITION = [2.0, 0.1, -3]
LOOK_AT = (-0.004, -0.05, 6.0)
UP = (-1.0, 0.0, 0.0)


def surface_to_pcl(mesh, alg="poisson", no_points=10000):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    # if _DEBUG:
    #     mesh_info(mesh)
    # if not points is None:
    #     no_points = points
    # else:
    #     no_points = len(mesh.vertices)//point_factor
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

def obj_size(obj : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = obj.get_max_bound()
    min_size = obj.get_min_bound()
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s


def show_objects(objlist, name=""):
    "Show the object list"
    o3d.visualization.draw_geometries(objlist, window_name=name, width=1000, height=1000)

def show_objects_test(obj, name=""):
    "Show the object list"
    o3d.visualization.draw_geometries(obj, window_name=name, width=1000, height=1000,
                                       zoom=ZOOM, front=CAM_POSITION, lookat=LOOK_AT, up=[0,1,0])

def pcl2pic(objects, name="", outfile=None):
    "Make a jpg file from pcl"
    obj_center = objects[0].get_center()
    ob_size = obj_size(objects[0])
    vis = o3d.visualization.Visualizer()
    res = vis.create_window(visible = _DEBUG, window_name=name, width=1000, height=1000)
    if not res:
        print("create window result", res)
    for obj in objects:
        vis.add_geometry(obj)
    ctr = vis.get_view_control()
    if ctr is None:
        print("pcl2jpg cant get view_control", vis)
    if _DEBUG:
        print('object center', obj_center, "object size", ob_size)
        print("cam position:", CAM_POSITION, "zoom", ZOOM)
    ctr.set_front(CAM_POSITION)
    #ctr.set_lookat(LOOK_AT)
    ctr.set_lookat(obj_center)
    ctr.set_up(UP)
    ctr.set_zoom(ZOOM)
    ctr.set_front([2.0, 0.1, -3])

    #render
    opt = vis.get_render_option()
    opt.point_size = 3.0
    vis.run()
    if outfile:
        if _DEBUG:
            print(f"Witing to outfile {outfile}")
        vis.capture_screen_image("ud.png", do_render=True)

if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='show3d', description='Show one/two 3d files')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-a', '--axis', required=False, help="Show coord system", action='store_true')
    parser.add_argument('-c', '--color', required=False, help="Add Collor to object", action="store_true")
    parser.add_argument('-nc', '--no_color', required=False, help="Add black color to obj", action="store_true")
    parser.add_argument('-w', "--write", action="store", type=str, required=False, help="Save image to file", metavar="imagefilename")
    parser.add_argument('-r', action="store_true",required=False, help="Show with auto camera")
    parser.add_argument('file1', help="The first stl or pointcloud")
    parser.add_argument('file2', nargs="?", help="The second stl or pointcloud")
    #parser.add_argument('file3', nargs="*", help="More stl or pointcloud")
    args = parser.parse_args()

    _DEBUG=args.d
    _VERBOSE = args.v
    if _DEBUG:
        print(args)

    fil1 = Path(args.file1)
    if _VERBOSE:
        print(f"Showing: {fil1}")
    # check files exists
    if not fil1.exists():
        print(f"input file {fil1} does not exist")
        sys.exit(1)
    # check file types

    vobjects = []

    if fil1.suffix=='.stl':
        mymesh = o3d.io.read_triangle_mesh(str(fil1))
        # if not mesh.has_triangle_colors():
        if args.color:
            mymesh.paint_uniform_color((0,1,0))
        if not mymesh.has_vertex_normals():
            print("add normals")
            mymesh.compute_vertex_normals()
        if not mymesh.has_triangle_normals():
            mymesh.compute_triangle_normals()
        size = obj_size(mymesh)
        mypcl = surface_to_pcl(mymesh)
        vobjects.append(mypcl)
        #in_pcl = stl2pcl(mesh)
    elif fil1.suffix=='.ply':
        mypcl = o3d.io.read_point_cloud(str(fil1))
        if args.color:
            mypcl.paint_uniform_color((0,1,0))
        if args.no_color:
            mypcl.paint_uniform_color((0,0,0))
        size = obj_size(mypcl)
        vobjects.append(mypcl)
    else:
        print("Input file type error")
        sys.exit(1)

    window_name = fil1.name
    if args.file2:
        fil2 = Path(args.file2)
        if fil2.suffix=='.ply':
            pcl2 = o3d.io.read_point_cloud(str(fil2))
            if args.color:
                pcl2.paint_uniform_color((1,0,0))
            vobjects.append(pcl2)
        elif fil1.suffix=='.stl':
            mesh2 = o3d.io.read_triangle_mesh(str(fil2))
            if args.color:
                mesh2.paint_uniform_color((1,0,0))
            vobjects.append(mesh2)
        else:
            print("Illegal file2 type")
            sys.exit(1)
        window_name += " - " + fil2.name

    if _VERBOSE:
        print("Object center", vobjects[0].get_center())
        print(f"Object size: {size} m")

    # for file in args.file3:
    #     fil = Path(file)
    #     print(fil.name)
    #     if fil.suffix=='.ply':
    #         pcl_n = o3d.io.read_point_cloud(str(fil))
    #         vobjects.append(pcl_n)
    #     elif fil.suffix=='.stl':
    #         mesh_n = o3d.io.read_triangle_mesh(str(fil))
    #         vobjects.append(mesh2)
    #     else:
    #         print("Illegal file2 type")
    #         sys.exit(1)
    #     window_name += " - " + fil.name

    if args.axis:
        if size > 1:
            ASIZE = 1
        else:
            ASIZE = 0.01
        print(ASIZE)
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=ASIZE,origin=(0,0,0))
        vobjects.append(coord)
    if _DEBUG:
        print("Number of objects:", len(vobjects))
        print(vobjects)
    if args.r:
        show_objects(vobjects)
    else:
        pcl2pic(vobjects, name=window_name, outfile=args.write)
