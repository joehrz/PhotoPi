# PhotoPi/point_cloud_analysis/point_cloud/hr_analysis.py

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import logging

from utils.helpers import cartesian_to_cylindrical
from point_cloud.convex_hull import (
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

    plant_height = np.max(z_cyl) - np.abs(np.min(z_cyl))
    max_r_index = np.argmax(r)
    max_r = r[max_r_index]
    max_r_theta = theta[max_r_index]
    max_r_z = z_cyl[max_r_index]

    # Plot Radius vs Theta
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    ax.scatter(theta, r, alpha=0.1, s=1, c='b', label='Point Cloud')
    ax.scatter(max_r_theta, max_r, c='r', s=10, label='Max R')
    plt.legend(loc='lower right')
    plt.title('Radius vs Theta of Plant')
    plt.show()

    # Find max and min heights
    max_H_index = np.argmax(z_cyl)
    max_H = z_cyl[max_H_index]
    max_H_r = r[max_H_index]
    max_H_theta = theta[max_H_index]

    min_H_index = np.argmin(z_cyl)
    min_H = z_cyl[min_H_index]
    min_H_r = r[min_H_index]

    # Plot Height vs Radius
    plt.figure()
    plt.scatter(r, z_cyl, alpha=0.1, s=1, c='b', label='Point Cloud')
    plt.scatter(max_H_r, max_H, c='r', s=10, label='Max Z')
    plt.scatter(min_H_r, min_H, c='r', s=10, label='Min Z')
    plt.xlabel('Radius')
    plt.ylabel('Height')
    plt.legend(loc='lower right')
    plt.title("Height vs Radius of Plant")
    plt.show()

    # Interactive Plot for Cursor Selection
    data_points = np.column_stack((r, z_cyl))
    fig, ax = plt.subplots()
    scatter = ax.scatter(data_points[:, 0], data_points[:, 1], alpha=0.1, s=1, c='blue')
    cursor, = ax.plot(0, 0, 'ro', markersize=5)
    move_with_mouse = True
    height = None  # Initialize height variable

    def on_move(event):
        """Update the cursor position only if move_with_mouse is True."""
        nonlocal move_with_mouse
        if move_with_mouse and event.xdata is not None and event.ydata is not None:
            cursor.set_data(event.xdata, event.ydata)
            fig.canvas.draw_idle()

    def on_click(event):
        """Find the nearest data point, set cursor to it, and print it."""
        nonlocal move_with_mouse, height
        move_with_mouse = not move_with_mouse

        if move_with_mouse:
            return

        if event.xdata is None or event.ydata is None:
            return

        distances = np.sum((data_points - np.array([event.xdata, event.ydata]))**2, axis=1)
        nearest_index = np.argmin(distances)
        nearest_point = data_points[nearest_index]

        # Update the cursor to the nearest point
        cursor.set_data(nearest_point[0], nearest_point[1])
        fig.canvas.draw_idle()

        print(f"Nearest data point: {nearest_point}")
        height = max_H - nearest_point[1]

    fig.canvas.mpl_connect('motion_notify_event', on_move)
    fig.canvas.mpl_connect('button_press_event', on_click)

    plt.show()

    if height is not None:
        print("Plant height based on selected point:", height)
        print("Real Plant Height:", height * scale)
    else:
        print("No height selected.")

    # Additional Calculations
    real_volume = compute_convex_hull_volume(points, scale=scale)
    print(f"Real Canopy Volume: {real_volume} cm³")

    # Visualization of Convex Hull
    visualize_convex_hull(point_cloud, color=(1, 0, 0))

    # Plot Volume Comparison
    plot_convex_hull_volume_comparison(points, scale=scale)

    analysis_results = {
        'plant_height_cm': plant_height * scale if height is not None else plant_height,
        'max_radius_cm': max_r * scale,
        'canopy_volume_real_cm3': real_volume,
        'height_to_radius_ratio': height / max_r if height is not None else None
    }

    return analysis_results


