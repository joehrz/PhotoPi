# PhotoPi/point_cloud_analysis/point_cloud/hr_analysis.py

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import logging

from photopack.point_cloud_analysis.utils.helpers import cartesian_to_cylindrical
from photopack.point_cloud_analysis.point_cloud.convex_hull import (
    compute_convex_hull_volume,
    visualize_convex_hull,
    plot_convex_hull_volume_comparison
)
# Configure logging (optional if already configured in main.py)
logger = logging.getLogger(__name__)

def get_top_percentage_with_indices(points, percentage):
    """
    Retrieve the top percentage of points based on the z-axis along with their indices.

    :param points: 3D point cloud as a numpy array (N, 3).
    :param percentage: Percentage of points to retrieve from the top.
    :return: Subset of points and their indices corresponding to the top percentage based on z-values.
    """
    sorted_indices = np.argsort(points[:, 2])
    num_points = int((percentage / 100) * sorted_indices.size)
    top_indices = sorted_indices[-num_points:]
    top_points = points[top_indices]
    return top_points, top_indices

def analyze_hr(point_cloud, scale):
    """
    Perform H/R (Height/Radius) analysis on the point cloud.

    Args:
        point_cloud (open3d.geometry.PointCloud): The processed point cloud.
        scale (float): Scaling factor based on turntable diameter.

    Returns:
        dict: Dictionary containing analysis results.
    """
    points = np.asarray(point_cloud.points)
    r, theta, z_cyl = cartesian_to_cylindrical(points[:, 0], points[:, 1], points[:, 2])

    # Compute z_min using a percentile to exclude outliers
    z_min_percentile = 10  # Adjust this value based on your data
    z_min = np.percentile(z_cyl, z_min_percentile)
    z_max = np.max(z_cyl)
    plant_height = z_max - z_min

    # Plot Histogram of Z-Values
    plt.figure()
    plt.hist(z_cyl, bins=100, color='blue', alpha=0.7)
    plt.axvline(z_min, color='red', linestyle='dashed', linewidth=2, label=f'Z Min ({z_min_percentile}th Percentile)')
    plt.xlabel('Z-Value (Height)')
    plt.ylabel('Frequency')
    plt.title('Distribution of Z-Values')
    plt.legend()
    plt.show()

    # Find max height details
    max_H_index = np.argmax(z_cyl)
    max_H = z_cyl[max_H_index]
    max_H_r = r[max_H_index]
    max_H_theta = theta[max_H_index]

    # Plot Height vs Radius
    plt.figure()
    plt.scatter(r, z_cyl, alpha=0.1, s=1, c='b', label='Point Cloud')
    plt.scatter(max_H_r, max_H, c='r', s=10, label='Max Z')
    plt.axhline(z_min, color='green', linestyle='dashed', linewidth=2, label=f'Z Min ({z_min_percentile}th Percentile)')
    plt.xlabel('Radius')
    plt.ylabel('Height')
    plt.legend(loc='lower right')
    plt.title("Height vs Radius of Plant")
    plt.show()

    # Compute Convex Hull Volume using SciPy
    real_volume = compute_convex_hull_volume(points, scale=scale)
    logger.info(f"Real Canopy Volume: {real_volume:.2f} cm³")

    # Visualization of Convex Hull
    visualize_convex_hull(point_cloud, color=(1, 0, 0))

    # Plot Volume Comparison
    plot_convex_hull_volume_comparison(points, scale=scale)

    analysis_results = {
        'plant_height_cm': plant_height * scale,
        'max_radius_cm': np.max(r) * scale,
        'canopy_volume_real_cm3': real_volume,
        'height_to_radius_ratio': plant_height / np.max(r) if np.max(r) != 0 else None
    }

    return analysis_results



