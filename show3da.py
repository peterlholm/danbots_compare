"Show 2 3d pictures"
import sys
from pathlib import Path
import argparse
import open3d as o3d
#import numpy as np


_DEBUG = True
_VERBOSE=True
ZOOM = 1
CAM_POSITION = (1.0, 1.0, -3.0)
CAM_POSITION = [5.0, 5.0, 5.0]

LOOK_AT = (-0.004, -0.05, 6.0)
LOOK_AT = (-0.004, -0.05, 6.0)
UP = (0.0,1.0,0.0)

def pcl2pic(pcd, outfile):
    "Make a jpg file from pcl"
    obj_center = pcd[0].get_center()
    vis = o3d.visualization.Visualizer()
    res = vis.create_window(visible = _DEBUG, window_name='png', width=1000, height=1000)
    if not res:
        print("create window result", res)
    for obj in pcd:
        vis.add_geometry(obj)

    ctr = vis.get_view_control()
    if ctr is None:
        print("pcl2jpg cant get view_control", vis)
    # parameters = o3d.io.read_pinhole_camera_parameters("camera_parameters.json")
    # ok = ctr.convert_from_pinhole_camera_parameters(parameters)
    # print(parameters.extrinsic)
    # print("ok", ok)
    # trajectory = o3d.io.read_pinhole_camera_parameters('camera_view.json')
    # ctr.convert_from_pinhole_camera_parameters(trajectory)
    if _DEBUG:
        print('object center', obj_center, "cam position:", CAM_POSITION, "zoom", ZOOM)
    ctr.set_front(CAM_POSITION)
    #ctr.set_lookat(LOOK_AT)
    ctr.set_up(UP)
    #ctr.set_zoom(ZOOM)
    #render


    opt = vis.get_render_option()
    # opt.point_size = 2.0
    opt.load_from_json('render_options.json')
    # #opt.show_coordinate_frame = True
    #opt.point_color_option = o3d.visualization.PointColorOption.Color
    #vis.update_renderer()
    if _DEBUG:
        vis.run()
        # param = vis.get_view_control().convert_to_pinhole_camera_parameters()
        # o3d.io.write_pinhole_camera_parameters("camera_param.json", param)
    vis.capture_screen_image(str(outfile), do_render=True)
    # if _DEBUG:
    #     img = vis.capture_screen_float_buffer(True)
    #     plt.imshow(np.asarray(img))
    #if not _TEXT:

def show_objects(obj):
    "Show the object list"
    #o3d.visualization.draw_geometries(obj, window_name="Navn", width=1000, height=1000)
    o3d.visualization.draw_geometries(obj, window_name="Navn", width=1000, height=1000,
                                      zoom=ZOOM, front=CAM_POSITION, lookat=LOOK_AT, up=[0,1,0])
                                  #zoom=0.3412,
                                  #zoom=0.63,
                                  #front=[0.4257, -0.2125, -0.8795],
                                  #front=[-10, 0, -40.8795],
                                  #lookat=[2.6172, 2.0475, 1.532],
                                  #lookat=[0, 0, 6],
                                  #lookat=[0, 0, 10.532],
                                  #up=[-0.0694, -0.9768, 0.2024])
                                  #up=[-10.0694, 0, 0.0])



if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='show3d', description='Show two 3d files')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-a', '--axis', required=False, help="Show coord system", action='store_true')
    parser.add_argument('file1', help="The first stl or pointcloud")
    parser.add_argument('file2', nargs="?", help="The second stl or pointcloud")
    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _VERBOSE = args.v
    print("Verbose output:", _VERBOSE)
    fil1 = Path(args.file1)

    if _VERBOSE:
        print(f"Showing {fil1}")
    # check files exists
    if not fil1.exists():
        print("input file(s) does not exist")
        sys.exit(1)
    # check file types

    objects = []

    if fil1.suffix=='.stl':
        mesh = o3d.io.read_triangle_mesh(str(fil1))
        # if not mesh.has_triangle_colors():
        mesh.paint_uniform_color((0,1,0))
        if not mesh.has_vertex_normals():
            print("add normals")
            mesh.compute_vertex_normals()
        if not mesh.has_triangle_normals():
            mesh.compute_triangle_normals()
        objects.append(mesh)
        #in_pcl = stl2pcl(mesh)
    elif fil1.suffix=='.ply':
        pcl = o3d.io.read_point_cloud(str(fil1))
        pcl.paint_uniform_color((0,1,0))
        objects.append(pcl)
    else:
        print("Input file type error")
        sys.exit(1)

    if args.file2:
        fil2 = Path(args.file2)
        if fil2.suffix=='.ply':
            pcl2 = o3d.io.read_point_cloud(str(fil2))
            pcl2.paint_uniform_color((1,0,0))
            objects.append(pcl2)
        elif fil1.suffix=='.stl':
            mesh2 = o3d.io.read_triangle_mesh(str(fil2))
            mesh2.paint_uniform_color((1,0,0))
            objects.append(mesh2)
        else:
            print("Illegal file2 type")
            sys.exit(1)
    if _VERBOSE:
        print("Object center", objects[0].get_center())
    #print(f"fil1: {fil1} fil2: {fil2}")
    if args.axis:
        ax = o3d.geometry.TriangleMesh.create_coordinate_frame() #size=0.01, origin=(0,0,0)
        objects.append(ax)
    print(objects)
    pcl2pic(objects, "test.png")
    show_objects(objects)
