# point_cloud_analysis/point_cloud/processing.py

import os
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from scipy.spatial import ConvexHull

from point_cloud_analysis.utils.helpers import (
    filter_non_points,
    normalize_vector,
    get_normal_vector,
    ensure_normal_points_toward_centroid,
    align_points_with_z_axis,
    translate_plane_to_origin
)

# Constants
TURNTABLE_DIAMETER = 39.878  # Diameter of the turntable in centimeters

def process_point_cloud(filename):
    """
    Load a point cloud, center it, segment a plane, and align the normal of the plane with the Z-axis.

    Args:
        filename (str): The path to the point cloud file.

    Returns:
        tuple: Processed point cloud and coordinate frame for visualization.
    """
    # Load the point cloud
    try:
        pcd = o3d.io.read_point_cloud(filename)
    except Exception as e:
        print(f"Error reading file {filename}: {e}")
        return None, None

    plant_name = os.path.splitext(os.path.basename(filename))[0].split('_fused_output_clean')[0]

    # Create a coordinate frame for visualization
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

    # Extract points and colors
    pcd_points = np.asarray(pcd.points)
    pcd_colors = np.asarray(pcd.colors)

    # Calculate centroid
    centroid = np.mean(pcd_points[:, :2], axis=0)
    print(f"Centroid (X, Y): {centroid}")

    # Center the point cloud
    pcd_points[:, 0] -= centroid[0]
    pcd_points[:, 1] -= centroid[1]
    pcd.points = o3d.utility.Vector3dVector(pcd_points)

    new_centroid = np.mean(pcd_points[:, :2], axis=0)
    print(f"New Centroid (X, Y): {new_centroid}")

    # Define color thresholds (example values, adjust as needed)
    lower = np.array([0.03529412, 0.3254902, 0.8])
    upper = np.array([0.09411765, 0.41176471, 0.89411765])

    # Filter non-target points based on color
    rest_points, rest_idx = filter_non_points(pcd_points, pcd_colors, lower, upper)
    rest_colors = pcd_colors[rest_idx]

    # Create a new point cloud with filtered points
    fix_pcd = o3d.geometry.PointCloud()
    fix_pcd.points = o3d.utility.Vector3dVector(rest_points)
    fix_pcd.colors = o3d.utility.Vector3dVector(rest_colors)
    fix_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[rest_idx])

    # Segment the largest plane in the point cloud
    plane_model, inliers = fix_pcd.segment_plane(distance_threshold=0.01,
                                                ransac_n=3,
                                                num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # Extract inlier and outlier point clouds
    plane_cloud = fix_pcd.select_by_index(inliers)
    outlier_cloud = fix_pcd.select_by_index(inliers, invert=True)

    # Get and adjust the normal vector
    normal_vector = get_normal_vector(plane_model)
    normal_vector = ensure_normal_points_toward_centroid(
        normal_vector,
        np.mean(np.asarray(plane_cloud.points), axis=0),
        np.mean(np.asarray(outlier_cloud.points), axis=0)
    )

    # Align the outlier cloud with the Z-axis
    rotated_points = align_points_with_z_axis(np.asarray(outlier_cloud.points), normal_vector)
    outlier_cloud.points = o3d.utility.Vector3dVector(rotated_points)
    outlier_cloud = translate_plane_to_origin(outlier_cloud, plane_model)

    # Visualize the segmented plane and outliers
    o3d.visualization.draw_geometries([plane_cloud, outlier_cloud, coordinate_frame])

    return outlier_cloud, coordinate_frame
