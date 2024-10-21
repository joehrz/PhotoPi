# PhotoPi/point_cloud_analysis/point_cloud/convex_hull.py

import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

def compute_convex_hull_volume(points, scale=1.0):
    """
    Computes the volume of the convex hull formed by the given points using SciPy.

    Args:
        points (np.ndarray): Array of point coordinates (N, 3).
        scale (float, optional): Scaling factor to apply to the points before volume calculation. Defaults to 1.0.

    Returns:
        float: Volume of the convex hull.
    """
    if len(points) < 4:
        print("Not enough points to compute a convex hull.")
        return 0.0
    hull = ConvexHull(points * scale)
    return hull.volume

def visualize_convex_hull(point_cloud, color=(1, 0, 0), coordinate_frame=None):
    """
    Visualizes the point cloud along with its convex hull using Open3D.

    Args:
        point_cloud (o3d.geometry.PointCloud): The point cloud to visualize.
        color (tuple, optional): RGB color for the convex hull lines. Defaults to (1, 0, 0).
        coordinate_frame (o3d.geometry.TriangleMesh, optional): Coordinate frame for reference. Defaults to None.
    """
    points = np.asarray(point_cloud.points)
    
    if len(points) < 4:
        print("Not enough points to compute a convex hull for visualization.")
        hull_ls = None
    else:
        # Compute the convex hull using Open3D
        hull, _ = point_cloud.compute_convex_hull()
        hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
        hull_ls.paint_uniform_color(color)
    
    geometries = [point_cloud]
    if hull_ls:
        geometries.append(hull_ls)
    if coordinate_frame:
        geometries.append(coordinate_frame)
    
    o3d.visualization.draw_geometries(geometries)

def plot_convex_hull_volume_comparison(points, scale=1.0):
    """
    Plots a comparison of the convex hull volume before and after scaling using SciPy.

    Args:
        points (np.ndarray): Array of point coordinates (N, 3).
        scale (float, optional): Scaling factor to apply. Defaults to 1.0.
    """
    original_volume = compute_convex_hull_volume(points, scale=1.0)
    scaled_volume = compute_convex_hull_volume(points, scale=scale)
    
    labels = ['Original Volume', f'Scaled Volume (x{scale})']
    volumes = [original_volume, scaled_volume]
    
    plt.figure(figsize=(8, 6))
    bars = plt.bar(labels, volumes, color=['blue', 'green'])
    plt.ylabel('Volume')
    plt.title('Convex Hull Volume Comparison')
    
    # Adding text labels on top of the bars
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height,
                 f'{height:.2f}', ha='center', va='bottom')
    
    plt.show()



