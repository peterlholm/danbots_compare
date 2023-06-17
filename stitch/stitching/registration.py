"pointcloud registrations"
import copy
import open3d as o3d
import numpy as np

_DEBUG = False
_SHOW = False
_TMPFILE = True

GLOBAL_FITNESS = 0.5
GLOBAL_RMSE = 0.001
LOCAL_FITNESS = 0.5
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
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])
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
    if _TMPFILE:
        o3d.io.write_point_cloud("/tmp/normals.ply", pcd_down)

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
    if _DEBUG:
        print("org voxel", voxel_size)
    voxel_size = 0.0005
    if _DEBUG:
        print("voxel set to", voxel_size)
    ref_down, test_down, ref_fpfh, test_fpfh = prepare_dataset(ref, test_target, voxel_size)
    if _DEBUG:
        o3d.visualization.draw_geometries([ref_down, test_down], window_name="downsample", width=1000, height=1000)
    result_ransac = execute_global_registration(
            ref_down, test_down, ref_fpfh, test_fpfh,
            voxel_size)
    if _DEBUG:
        print("global transformation matrix", result_ransac, np.around(result_ransac.transformation,3))
        draw_registration_result(ref_down, test_down, result_ransac.transformation, window_name="Global registration")
    if result_ransac.fitness < GLOBAL_FITNESS or result_ransac.inlier_rmse > GLOBAL_RMSE:
        print("BAD GLOBAL REGISTRATION", result_ransac)
    result_icp = execute_local_registration(
            test_down, ref_down,
            voxel_size, result_ransac.transformation)
    if _DEBUG:
        print("Local transformation matrix", result_icp, np.around(result_icp.transformation,3))
        draw_registration_result(ref_down, test_down, result_icp.transformation, window_name="Local registration downsample")
    if result_icp.fitness < LOCAL_FITNESS or result_icp.inlier_rmse >LOCAL_RMSE:
        print("BAD LOCAL REGISTRATION", result_icp)
        return None, None
    if _DEBUG:
        draw_registration_result(ref, test_target, result_icp.transformation, window_name="Local registration originals")

    transformation = result_icp.transformation

    return test_target, transformation #, inf_matrix
