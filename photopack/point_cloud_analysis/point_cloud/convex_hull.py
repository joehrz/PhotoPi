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





