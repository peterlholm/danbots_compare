#!/bin/python3
"Stitch a pointcloud to pointcloud"
import sys
import math
import copy
import subprocess
from pathlib import Path
import argparse
import numpy as np
import open3d as o3d

_DEBUG = False
_SHOW = True
_VERBOSE = False

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

def stl2pcl(mesh):
    "create pointcloud from vertices in stl"
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    return pcd

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


# NOICE REMOVAL

# Limit is ratio of total points in a cluster required to keep it.
def keep_significant_clusters(pcd, limit=0.06, eps=0.35, min_points=7):
    "reduse pointcloud to significant clusters"
    if _DEBUG:
        print(f"keeep significant cluster DBSCAN limit {limit} eps {eps} min_points {min_points}")
    pcd_result = o3d.geometry.PointCloud()
    clusters = pcd.cluster_dbscan(eps, min_points)
    if _DEBUG:
        print("DBSCAN result cluster[0]", clusters[0])
    # Messy way to count how many points are in each cluster.
    cluster_indicies = np.array(clusters) + 1
    # Bincount only counts non-negative.
    cluster_indicies_count = np.bincount(cluster_indicies)
    ii_vec = np.nonzero(cluster_indicies_count)[0]
    # (ii_vec - 1) corrects the one added above.
    counts = zip(ii_vec-1, cluster_indicies_count[ii_vec])
    # if _DEBUG:
    #     print("counts", len(counts))
    kept_indicies = []
    for (cluster, count) in counts:
        if cluster == -1:  # Skip the noise.
            if _DEBUG:
                print("skip", count)
            continue
        # if _DEBUG:
        #     print(count)
        kept = count / len(pcd.points)
        if kept >= limit:
            #print("kept", kept, "limit", limit)
            indicies = get_cluster_indicies(clusters, cluster)
            #print("inde", indicies)
            kept_indicies += indicies
            #print("kept:ind", kept_indicies)
            pcd_result += pcd.select_by_index(indicies)
            if _DEBUG:
                print("kept", kept, "limit", limit)
                #print("inde", indicies)
                #print("kept:ind", kept_indicies)
                #print("inserted")
        else:
            pass
    if _DEBUG:
        print(f"no points {len(pcd.points)} kept indicies  {len(kept_indicies)}")
    return (pcd_result, kept_indicies)


def get_cluster_indicies(clusters, cluster):
    "get cluster image"
    #print("get_cluster_indices")
    return [i for i, x in enumerate(clusters) if x == cluster]

# downsample

#VOXEL_SIZE = 0.0001
VOXEL_SIZE = 0.0005

# registration

GLOBAL_FITNESS = 0.6
GLOBAL_RMSE = 0.001
#GLOBAL_FITNESS = 0.2
#GLOBAL_RMSE = 0.001
LOCAL_FITNESS = 0.2
LOCAL_RMSE = 0.001


def draw_registration_result(reference, test_source, transformation, axis=False, window_name="registration result", color=False):
    "Debug draw registration result"
    reference_temp = copy.deepcopy(reference)
    test_temp = copy.deepcopy(test_source)
    if color:
        reference_temp.paint_uniform_color([0, 0.7, 0.1])
        test_temp.paint_uniform_color([1, 0.0, 0.1])
    test_temp.transform(transformation)
    pointclouds =[reference_temp, test_temp]
    if axis:
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
        pointclouds.append(axis_pcd)
    o3d.visualization.draw_geometries(pointclouds, window_name=window_name, width=1000, height=1000)

def compute_normal(pcd):
    "creates normalts that all point in same (wrong) direction (due to low radius)"
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
              radius=0.1, max_nn=30))
    normals_load = np.asarray(pcd.normals) * -1  # Flip normals.
    pcd.normals = o3d.utility.Vector3dVector(normals_load)
    # Get new and correctly orientated normals.
    pcd.estimate_normals()

def preprocess_point_cloud(pcd, voxel_size):
    "prepare point cloud by down sample and compute features"
    pcd_down = pcd.voxel_down_sample(voxel_size)
    if _DEBUG:
        print(f"Downsample voxel({voxel_size }) to {len(pcd_down.points)} points")
    compute_normal(pcd_down)
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    if _DEBUG:
        print(f"pcd_fpfh features dimension: {pcd_fpfh.dimension()} numbers: {pcd_fpfh.num()}")
    return pcd_down, pcd_fpfh

def execute_global_registration(reference_down, target_down,
                                reference_fpfh, target_fpfh,
                                voxel_size, dist_thres_scalar=1.5,
                                scale=False, edge_length_thres=0.99,
                                converge_itr=(10**8),
                                converge_certainty=0.9999):
    "find global registation"

    distance_threshold = voxel_size * dist_thres_scalar
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        target_down, reference_down,  target_fpfh, reference_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(scale),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                edge_length_thres),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(
            converge_itr, converge_certainty))
    return result

def execute_local_registration(source_down, reference_down, voxel_size,
                               init_transformation, converge_max_itr=30):
    "Find local registration"
    conver_crit = o3d.pipelines.registration.ICPConvergenceCriteria()
    conver_crit.max_iteration = converge_max_itr
    result_icp = o3d.pipelines.registration.registration_icp(
                    source_down, reference_down, voxel_size, init_transformation,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=conver_crit)
    return result_icp

def prepare_dataset(ref, test_target, voxel_size):
    "prepare data set "
    ref_down, ref_fpfh = preprocess_point_cloud(ref, voxel_size)
    test_target_down, test_target_fpfh = preprocess_point_cloud(test_target, voxel_size)
    return ref_down, test_target_down, ref_fpfh, test_target_fpfh

def get_transformations(ref, test_target, voxel_size):
    "get transformations from pointclouds"
    ref_down, test_down, ref_fpfh, test_fpfh = prepare_dataset(ref, test_target, voxel_size)
    if _DEBUG:
        o3d.visualization.draw_geometries([ref_down, test_down], window_name="downsample", width=1000, height=1000)
    result_ransac = execute_global_registration(
            ref_down, test_down, ref_fpfh, test_fpfh,
            voxel_size)
    if _VERBOSE:
        print(f"Global registration result: Fitness: {result_ransac.fitness:.2f} RMSE: {result_ransac.inlier_rmse:.6f}")
    if _DEBUG:
        print("global transformation matrix", result_ransac, np.around(result_ransac.transformation,3))
        print("Transformation Matrix\n", result_ransac.transformation)
        draw_registration_result(ref_down, test_down, result_ransac.transformation, window_name="Global registration")
    if result_ransac.fitness < GLOBAL_FITNESS or result_ransac.inlier_rmse > GLOBAL_RMSE:
        print(f"BAD GLOBAL REGISTRATION Fitness: {result_ransac.fitness:.2f} RMSE: {result_ransac.inlier_rmse:.6f}")
        return False, None

    result_icp = execute_local_registration(
            test_down, ref_down,
            voxel_size, result_ransac.transformation)
    if _VERBOSE:
        print(f"Local registration result: Fitness: {result_icp.fitness:.2f} RMSE: {result_icp.inlier_rmse:.6f}")
    if _DEBUG:
        print("Local transformation matrix", result_icp, np.around(result_icp.transformation,3))
        draw_registration_result(ref_down, test_down, result_icp.transformation, window_name="Local registration downsample")
    if result_icp.fitness < LOCAL_FITNESS or result_icp.inlier_rmse >LOCAL_RMSE:
        print(f"BAD LOCAL REGISTRATION Fitness: {result_icp.fitness:.2f} RMSE: {result_icp.inlier_rmse:.6f}")
        return None, None
    if _DEBUG:
        draw_registration_result(ref, test_target, result_icp.transformation, window_name="Local registration originals")

    transformation = result_icp.transformation
    return test_target, transformation #, inf_matrix


# stitching
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
    pcd_result, kept_indicies = keep_significant_clusters(pcd, required_share, epsilon, minimum_points)
    if _DEBUG:
        print(f"Kept points: {len(kept_indicies)} Removing  {len(pcd.points) - len(kept_indicies)}")
    return pcd_result

def reg_point_clouds(ref, new):
    "register point cloud and find tranformatin bringing new to ref"
    test_target, transformation = get_transformations(ref, new, VOXEL_SIZE)
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
    if test_target is False:
        return None
    if debug:
        print("Regisering test_target", test_target)
        print("Regisering transformation:", transformation)
    if debug:
        print("Transformation", transformation)
    return transformation

def trans_file(pcl, outfile, transformation):
    "transform and write file with transformation"
    pcl.transform(transformation)
    o3d.io.write_point_cloud(str(outfile), pcl)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog='stitch3d', description='Stitch two 3d files and calculate error figures. Return 1 if registration fails')
    parser.add_argument('-d', required=False, help="Turn debug on", action='store_true' )
    parser.add_argument('-v', required=False, help="Give verbose output", action='store_true' )
    parser.add_argument('-s', '--show', required=False, help="Show the stitch result", action='store_true' )
    parser.add_argument('-c', '--clean', required=False, help="Clean the point clounds", action='store_true' )
    parser.add_argument('-a', '--axis', required=False, help="Add axis to pictures", action='store_true' )
    parser.add_argument('org_file', type=Path, help="The original stl or pointcloud")
    parser.add_argument('test_file', type=Path, help="The pointcloud to be measured")
    parser.add_argument('-o', '--outfile', required=False, nargs=1, help="Make an output file", action='store' )
    parser.add_argument('-e', '--error', required=False, help="calculate error with compare", action='store_true' )
    args = parser.parse_args()

    _DEBUG = args.d
    _VERBOSE = args.v
    if _VERBOSE:
        print(f"Stitching {str(args.org_file)} and {args.test_file}")
    # check files exists
    if not args.org_file.exists() or not args.test_file.exists():
        print("input file(s) does not exist")
        sys.exit(9)
    # check file types
    if args.org_file.suffix=='.stl':
        inmesh = o3d.io.read_triangle_mesh(str(args.org_file))
        obj_size = mesh_size(inmesh)
        if _VERBOSE:
            print(f"Input object size {obj_size:.2f}")
        in_pcl = surface_to_pcl(inmesh)
    elif args.org_file.suffix=='.ply':
        in_pcl = o3d.io.read_point_cloud(str(args.org_file))
    else:
        print("Input file type error")
        sys.exit(8)

    if args.test_file.suffix=='.ply':
        t_pcl = o3d.io.read_point_cloud(str(args.test_file))
    else:
        print("Input file2 type error")
        sys.exit(8)
    #
    ot_pcl = copy.deepcopy(t_pcl)
    transform = stitch_trans(in_pcl, t_pcl, debug=args.d, use_cleaning=args.clean)
    if transform is None:
        print("No transformation found")
        sys.exit(1)
    print("Resulting Transformation:\n", transform)
    #
    trans, t_rot, t_scale = decode_transformation(transform)
    print(f"Translation: {trans}\nRotation: {t_rot}\nScale {t_scale}" )
    if transform is None:
        print("No transformation can be found")
        sys.exit(1)
    if args.outfile:
        print("Saving result to:", args.outfile[0])
        trans_file(t_pcl, args.outfile[0], transform)
    if args.show:
        draw_registration_result(in_pcl, ot_pcl, transform, window_name="Stitch result", color=True, axis=args.axis)

    if args.error and args.outfile:
        print("-- comparing original and test --")
        cmd = ["compare3d", str(args.org_file), str(args.test_file)]
        subprocess.run(cmd, check=False)
        print("-- comparing original and result --")
        cmd = ["compare3d", str(args.org_file), args.outfile[0]]
        subprocess.run(cmd, check=False)
