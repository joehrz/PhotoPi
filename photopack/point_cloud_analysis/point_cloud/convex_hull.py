# PhotoPi/point_cloud_analysis/point_cloud/convex_hull.py

import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

class ConvexHullAnalyzer:
    def __init__(self, point_cloud):
        """
        Initializes the ConvexHullAnalyzer with a point cloud.

        Args:
            point_cloud (o3d.geometry.PointCloud): The point cloud to analyze.
        """
        self.point_cloud = point_cloud
        self.points = np.asarray(point_cloud.points)
        self.volume = None

    def compute_convex_hull_volume(self, scale=1.0):
        """
        Computes the volume of the convex hull formed by the point cloud using SciPy.

        Args:
            scale (float, optional): Scaling factor to apply to the points before volume calculation. Defaults to 1.0.

        Returns:
            float: Volume of the convex hull.
        """
        points = self.points * scale

        if len(points) < 4:
            print("Not enough points to compute a convex hull.")
            self.volume = 0.0
            return self.volume

        hull = ConvexHull(points)
        self.volume = hull.volume
        return self.volume

    def visualize_convex_hull(self, color=(1, 0, 0), coordinate_frame=None):
        """
        Visualizes the point cloud along with its convex hull using Open3D.

        Args:
            color (tuple, optional): RGB color for the convex hull lines. Defaults to (1, 0, 0).
            coordinate_frame (o3d.geometry.TriangleMesh, optional): Coordinate frame for reference. Defaults to None.
        """
        points = self.points

        if len(points) < 4:
            print("Not enough points to compute a convex hull for visualization.")
            hull_ls = None
        else:
            # Compute the convex hull using Open3D
            hull_mesh, _ = self.point_cloud.compute_convex_hull()
            hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull_mesh)
            hull_ls.paint_uniform_color(color)

        geometries = [self.point_cloud]
        if hull_ls:
            geometries.append(hull_ls)
        if coordinate_frame:
            geometries.append(coordinate_frame)

        o3d.visualization.draw_geometries(geometries)

    def process_pc_percentile_volume(self, percentage, scale=1.0, color=(1, 0, 0), visualize=True):
        """
        Processes the point cloud to compute the convex hull volume for the top percentile of points based on the z-axis.

        Args:
            percentage (float): Percentage of top points to consider based on z-values.
            scale (float, optional): Scaling factor to apply to the points before volume calculation. Defaults to 1.0.
            color (tuple, optional): RGB color for the convex hull lines. Defaults to (1, 0, 0).
            visualize (bool, optional): Whether to visualize the convex hull. Defaults to True.

        Returns:
            float: Volume of the convex hull for the top percentile of points.
        """
        if not (0 < percentage <= 100):
            raise ValueError("Percentage must be between 0 and 100.")

        # Retrieve the top percentage of points based on the z-axis
        top_points, top_indices = self.get_top_percentage_with_indices(self.points, percentage)

        if len(top_points) < 4:
            print("Not enough points in the top percentile to compute a convex hull.")
            return 0.0

        # Apply scaling
        scaled_top_points = top_points * scale

        # Compute the convex hull using SciPy
        hull = ConvexHull(scaled_top_points)
        percentile_volume = hull.volume

        self.volume = percentile_volume

        if visualize:
            # Create a new point cloud for the top percentile
            point_cloud_z = o3d.geometry.PointCloud()
            point_cloud_z.points = o3d.utility.Vector3dVector(top_points)

            # If the original point cloud has colors, apply them to the subset
            if self.point_cloud.has_colors():
                original_colors = np.asarray(self.point_cloud.colors)
                top_colors = original_colors[top_indices]
                point_cloud_z.colors = o3d.utility.Vector3dVector(top_colors)
            else:
                # Assign a uniform color if no colors are present
                point_cloud_z.paint_uniform_color(color)

            # Compute convex hull for visualization using Open3D
            hull_mesh, _ = point_cloud_z.compute_convex_hull()
            hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull_mesh)
            hull_ls.paint_uniform_color(color)

            # Visualize the top percentile point cloud and its convex hull
            o3d.visualization.draw_geometries([point_cloud_z, hull_ls],
                                              window_name=f"Top {percentage}% Convex Hull",
                                              width=800, height=600)

        return percentile_volume


    def get_top_percentage_with_indices(self, points, percentage):
        """
        Retrieve the top percentage of points based on the z-axis along with their indices.

        Args:
            points (np.ndarray): 3D point cloud as a numpy array (N, 3).
            percentage (float): Percentage of points to retrieve from the top.

        Returns:
            tuple: (Subset of points, their corresponding indices)
        """
        # Sort the indices based on the z-axis
        sorted_indices = np.argsort(points[:, 2])

        # Calculate the number of points for the given percentage
        num_points = int((percentage / 100) * sorted_indices.size)

        # Retrieve the indices for the top percentage of points
        top_indices = sorted_indices[-num_points:]

        # Use the top indices to get the corresponding points
        top_points = points[top_indices]

        return top_points, top_indices





