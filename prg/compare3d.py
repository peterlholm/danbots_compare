#!/bin/python3
"Compare a pointcloud to mesh"
import sys
from pathlib import Path
import argparse
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

_DEBUG = False
_SHOW = False
_VERBOSE = False

MY = "µm"

def mesh_size(mesh : o3d.geometry.TriangleMesh):
    "calculate average size of mesh"
    max_size = mesh.get_max_bound()
    min_size = mesh.get_min_bound()
    s = 0
    for i in range(2):
        s += max_size[i] - min_size[i]
    s = s / 3
    return s

def mesh_info(mesh):
    "print interesting info about mesh"
    print("Bounding box",mesh.get_axis_aligned_bounding_box())
    print("Oriented Bounding box",mesh.get_oriented_bounding_box())
    print("Vertices", len(mesh.vertices))

def surface_to_pcl(mesh, alg="poisson", number_of_points=100000, init_factor=10):
    "convert mesh surfaces to pointcloud, point_factor vertices/points"
    if _DEBUG:
        mesh_info(mesh)
        print(f"Algoritm: {alg} Number_of_points: {number_of_points} Point_factor: {init_factor}")
    if alg=='poisson':
        pcl = mesh.sample_points_poisson_disk(number_of_points=number_of_points, )
    else:
        pcl = mesh.sample_points_uniformly(number_of_points=number_of_points, init_factor=init_factor)
    return pcl

def show_pcls(pcl1, pcl2, axis=False, name=""):
    "show pointcloud on screen"
    pcl1.paint_uniform_color([0,1,0])
    pcl2.paint_uniform_color([0.1,0.0,0])
    objects = [pcl1, pcl2]
    size = mesh_size(pcl1)
    if size > 5:
        asize = 10
    else:
        asize = 0.01
    #print ("Size", size)
    if axis:
        axi = o3d.geometry.TriangleMesh.create_coordinate_frame(size=asize, origin=(0,0,0))
        objects.append(axi)
    o3d.visualization.draw_geometries(objects, width=1000, height=1000, window_name=name)

def cmp2pcl(org_pcl, test_pcl):
    "compare 2 pcl a pointcloud and return a value for the error"
    if _DEBUG:
        print("No Points in reference", len(org_pcl.points))
        print("No Points in testfile", len(test_pcl.points))
    dist = test_pcl.compute_point_cloud_distance(org_pcl)
    #print(dist)
    distance = np.asarray(dist)
    if _DEBUG:
        print(distance)
    pclerror = np.sqrt(np.mean(distance ** 2))
    if _DEBUG:
        print(f"Min error:  {np.min(distance):.6f} m")
        print(f"Max error:  {np.max(distance):.6f} m")
        print(f"Mean error: {np.mean(distance):.6f} m")
        print(f"RMS:        {pclerror:.6f} m")
    return pclerror, np.min(distance), np.max(distance), np.mean(distance), distance

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='compare3d', description='Compare two 3d files and calculate error figures\n\nSee also conv3d, crop3d, info3d, mirror3d, show3d, trans3d, stitch3d')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-s', required=False, help="Show output", action='store_true' )
    parser.add_argument('-a', required=False, help="Show axis", action='store_true' )
    parser.add_argument('--histogram', required=False, help="Generate Histogram", action='store_true' )
    parser.add_argument('-o', '--output', required=False, type=Path, help="Error Histogram Output file", metavar="Outputfile")
    parser.add_argument('--script', required=False, help="Scripting output", action='store_true' )
    parser.add_argument('org_file', help="The reference stl or pointcloud")
    parser.add_argument('test_file', help="The test pointcloud to be measured")
    args = parser.parse_args()
    if args.d:
        _DEBUG=True
    _SHOW = args.s
    _VERBOSE = args.v
    fil1 = Path(args.org_file)
    fil2 = Path(args.test_file)
    if _VERBOSE:
        print(f"Comparing {fil1} and {fil2}")
    # check files exists
    if not fil1.exists() or not fil2.exists():
        print("input file(s) does not exist", file=sys.stderr)
        sys.exit(1)
    if fil1.suffix=='.stl':
        inmesh = o3d.io.read_triangle_mesh(str(fil1))
        in_pcl = surface_to_pcl(inmesh)
    elif fil1.suffix=='.ply':
        in_pcl = o3d.io.read_point_cloud(str(fil1))
    else:
        print("Input file type error", file=sys.stderr)
        sys.exit(1)

    if fil2.suffix=='.ply':
        t_pcl = o3d.io.read_point_cloud(str(fil2))
    elif fil1.suffix=='.stl':
        testmesh = o3d.io.read_triangle_mesh(str(fil2))
        t_pcl = surface_to_pcl(testmesh)
    else:
        print("Input file type error", file=sys.stderr)
        sys.exit(1)

    obj_size = mesh_size(in_pcl)

    if _VERBOSE:
        print(f"Reference object size {obj_size:.2f} m")
        print(f"Test object size {mesh_size(t_pcl):.2f} m")
        print("Points in reference", len(in_pcl.points))
        print("Points in testfile", len(t_pcl.points))

    rms, vmin, vmax, mean, distarr = cmp2pcl(in_pcl, t_pcl)

    if not args.script:
        print(f"Point ABS distance Error: RMSE: {rms*1000000:.0f} {MY} MinE: {vmin*1000000:.0f} {MY} MaxE: {vmax*1000000:.0f} {MY} MeanE: {mean*1000000:.0f} {MY}")
        if _DEBUG:
            print(f"Point ABS distance Error: RMS: {rms:.2e} m Min: {vmin:.2e} m Max: {vmax:.2e} m Mean: {mean:.2e} m")
        print(f"Point Rel distance Error: RMSE: {rms/obj_size*100:.3f}% MinE: {vmin/obj_size*100:.3f}% MaxE: {vmax/obj_size*100:.3f}% MeanE: {mean/obj_size*100:.3f}%")
    else:
        print(f"{rms:.6f} {vmin:.6f} {vmax:.6f} {mean:.6f} {rms/obj_size:.3f} {vmin/obj_size:.3f} {vmax/obj_size:.3f} {mean/obj_size:.3f}")

    if _SHOW:
        show_pcls(in_pcl, t_pcl, axis=args.a, name="Org pointclouds")

    if args.histogram or args.output:
        no_points = len(distarr)
        dist_u = distarr * 1000000
        hist_arr, bin_edges = np.histogram(dist_u)
        #fig, axs = plt.subplots(1, 2, sharey=True, tight_layout=True)
        fig, axs = plt.subplots()
        axs.set_title("Error distribution")
        axs.set_xlabel("Error in µm")
        axs.set_ylabel("Number points")
        POS_X = 0.75
        POS_Y = 0.95
        DY=-0.05
        # We can set the number of bins with the *bins* keyword argument.
        axs.hist(dist_u, bins=10)
        axs.text(POS_X, POS_Y, f"No points: {no_points}",transform=axs.transAxes)
        axs.text(POS_X, POS_Y+DY, f"RMSE: {rms*1000000:.0f} {MY}",transform=axs.transAxes)
        axs.text(POS_X, POS_Y+2*DY, f"MaxE: {vmax*1000000:.0f} {MY}",transform=axs.transAxes)
        axs.text(POS_X, POS_Y+3*DY, f"MinE: {vmin*1000000:.0f} {MY}",transform=axs.transAxes)
        #axs.text(POS_X, POS_Y+4*DY, f"MeanE: {mean*1000000:.0f} {MY}",transform=axs.transAxes)
        #axs[1].hist(dist2, bins=n_bins)
        if args.histogram:
            plt.show()
        if args.output:
            fig.savefig(args.output)
