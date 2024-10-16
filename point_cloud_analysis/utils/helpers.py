# point_cloud_analysis/utils/helpers.py

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

def filter_non_points(points, colors, lower_threshold, upper_threshold):
    """
    Filter points based on non-target color thresholds.

    Args:
        points (np.ndarray): Array of point coordinates (N, 3).
        colors (np.ndarray): Array of color values corresponding to points (N, 3).
        lower_threshold (np.ndarray): Lower RGB color threshold.
        upper_threshold (np.ndarray): Upper RGB color threshold.

    Returns:
        tuple: Filtered points and their indices.
    """
    mask = np.all((colors >= lower_threshold) & (colors <= upper_threshold), axis=1)
    filtered_points = points[mask]
    filtered_indices = mask
    return filtered_points, filtered_indices

def normalize_vector(vector):
    """
    Normalize a 3D vector.

    Args:
        vector (np.ndarray): The vector to normalize.

    Returns:
        np.ndarray: The normalized vector.
    """
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return vector / norm

def get_normal_vector(plane_model):
    """
    Extract the normal vector from a plane model.

    Args:
        plane_model (tuple): Plane equation coefficients (a, b, c, d).

    Returns:
        np.ndarray: Normal vector (a, b, c).
    """
    a, b, c, _ = plane_model
    return np.array([a, b, c])

def ensure_normal_points_toward_centroid(normal, centroid1, centroid2):
    """
    Ensure that the normal vector points from centroid1 towards centroid2.

    Args:
        normal (np.ndarray): The normal vector.
        centroid1 (np.ndarray): First centroid.
        centroid2 (np.ndarray): Second centroid.

    Returns:
        np.ndarray: Adjusted normal vector.
    """
    direction_vector = centroid2 - centroid1
    if np.dot(normal, direction_vector) < 0:
        return -normal
    return normal

def align_points_with_z_axis(points, normal_vector):
    """
    Rotate points so that the normal vector aligns with the Z-axis.

    Args:
        points (np.ndarray): Array of point coordinates (N, 3).
        normal_vector (np.ndarray): The normal vector to align with Z-axis.

    Returns:
        np.ndarray: Rotated points.
    """
    z_axis = np.array([0, 0, 1])
    rotation_axis = np.cross(normal_vector, z_axis)
    rotation_axis = normalize_vector(rotation_axis)
    angle = np.arccos(np.clip(np.dot(normal_vector, z_axis), -1.0, 1.0))

    # Handle the case when vectors are parallel or anti-parallel
    if np.linalg.norm(rotation_axis) < 1e-6:
        if np.dot(normal_vector, z_axis) > 0:
            return points  # No rotation needed
        else:
            return -points  # 180-degree rotation

    rotation_matrix = R.from_rotvec(angle * rotation_axis).as_matrix()
    rotated_points = np.dot(points, rotation_matrix.T)
    return rotated_points

def translate_plane_to_origin(point_cloud, plane_equation):
    """
    Translate the point cloud so that the plane aligns with z = 0.

    Args:
        point_cloud (open3d.geometry.PointCloud): The point cloud to translate.
        plane_equation (tuple): Plane equation coefficients (a, b, c, d).

    Returns:
        open3d.geometry.PointCloud: Translated point cloud.
    """
    a, b, c, d = plane_equation
    norm = np.linalg.norm([a, b, c])
    distance = d / norm

    # Translation vector to move the plane to z = 0
    translation = np.array([0, 0, -distance])

    # Apply translation
    translated_pcd = point_cloud.translate(translation, relative=True)
    return translated_pcd

def cartesian_to_cylindrical(x, y, z):
    """
    Convert Cartesian coordinates to Cylindrical coordinates.

    Args:
        x (array-like): X coordinates.
        y (array-like): Y coordinates.
        z (array-like): Z coordinates.

    Returns:
        tuple: (r, theta, z) in cylindrical coordinates.
    """
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    return r, theta, z

