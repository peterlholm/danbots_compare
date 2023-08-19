"Mirror 3d pictures"
import sys
from pathlib import Path
import argparse
import open3d as o3d
import numpy as np


def mirror_pcl_axis(infile, outfile, axis):
    "Mirror pcl about axis"
    pcd = o3d.io.read_point_cloud(str(infile))
    arr = np.asarray(pcd.points)
    # print("Input.colors", pcd.has_colors())
    # print(pcd.points[0])
    # print(pcd.colors[0])
    if axis=="y":
        #print('xyz_load', arr)
        for pkt in arr:
            pkt[1] = -pkt[1]
            #print(pkt)
        #print('xyz_load', arr)
    elif axis=="x":
        #print('xyz_load', arr)
        for pkt in arr:
            pkt[1] = -pkt[1]
            #print(pkt)
        #print('xyz_load', arr)
    else:
        print("Not implemented:", axis)
    opcd = o3d.geometry.PointCloud()
    opcd.points = o3d.utility.Vector3dVector(arr)
    #opcd.colors = pcd.colors
    opcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
    # print(opcd.points[0])
    # print(opcd.colors[0])

    # print("Output.colors", opcd.has_colors())
    o3d.io.write_point_cloud(str(outfile), opcd)



if __name__ == "__main__":
    #PICFOLDER = Path(__file__).parent / 'testdata'
    parser = argparse.ArgumentParser(prog='mirror3d', description='Mirror pcl round x')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-a-', '--axis', required=False, help="Show coord system", action='store_true')
    parser.add_argument('file1', help="The first stl or pointcloud")
    parser.add_argument('file2', nargs="?", help="The second stl or pointcloud")
    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _VERBOSE = args.v
  
    fil1 = Path(args.file1)
    fil2 = Path(args.file2)

    if _VERBOSE:
        print(f"Mirroring {fil1}")
    # check files exists
    if not fil1.exists():
        print("input file(s) does not exist")
        sys.exit(1)
    # check file types

    objects = []

    mirror_pcl_axis(fil1, fil2, axis="x")
    if fil1.suffix=='.ply':
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
        else:
            print("Illegal file2 type")
            sys.exit(1)
    if _VERBOSE:
        print("Object center", objects[0].get_center())
    #print(f"fil1: {fil1} fil2: {fil2}")
