"Show 2 3d pictures"
import sys
from pathlib import Path
import argparse
import open3d as o3d

def show_objects(obj):
    "Show the object list"
    o3d.visualization.draw_geometries(obj, window_name="Navn", width=1000, height=1000)
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

    #print(f"fil1: {fil1} fil2: {fil2}")

    print(objects)
    show_objects(objects)
