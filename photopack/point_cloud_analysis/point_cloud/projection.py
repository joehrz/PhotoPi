# photopack/point_cloud_analysis/point_cloud/projection.py

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import alphashape
from shapely.geometry import Polygon, MultiPolygon
import logging

# Configure logging
logger = logging.getLogger(__name__)

class ProjectionAnalyzer:
    def __init__(self, point_cloud):
        """
        Initializes the ProjectionAnalyzer with a point cloud.
        
        Args:
            point_cloud (open3d.geometry.PointCloud): The point cloud to analyze.
        """
        self.point_cloud = point_cloud
        self.points = np.asarray(point_cloud.points)
        self.alpha_shape = None
        self.alpha_shape_area = None

    def compute_alpha_shape_area(self, alpha=1.0):
        """
        Computes the alpha shape area of the point cloud projected onto the XY plane.
        
        Args:
            alpha (float, optional): Alpha parameter for alpha shape. Defaults to 1.0.
        
        Returns:
            float: The area of the alpha shape.
        """
        # Project points onto XY plane
        xy_points = self.points[:, :2]
        
        # Compute alpha shape
        self.alpha_shape = alphashape.alphashape(xy_points, alpha)
        
        if self.alpha_shape is None or self.alpha_shape.is_empty:
            logger.warning("No valid alpha shape found for the given points and alpha value.")
            self.alpha_shape_area = 0.0
            return self.alpha_shape_area
        
        # Calculate area
        self.alpha_shape_area = self.alpha_shape.area
        
        return self.alpha_shape_area

    def plot_original_points(self, save_fig=False, filename="original_points.png"):
        """
        Plots the original points projected onto the XY plane.
        
        Args:
            save_fig (bool, optional): Whether to save the figure. Defaults to False.
            filename (str, optional): Filename for saving the figure. Defaults to "original_points.png".
        """
        xy_points = self.points[:, :2]
        plt.figure()
        plt.plot(xy_points[:, 0], xy_points[:, 1], 'o', label='Points', markersize=3)
        plt.title("Original Points")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.legend()
        if save_fig:
            plt.savefig(filename)
        plt.show()

    def plot_alpha_shape(self, save_fig=False, filename="alpha_shape.png"):
        """
        Plots the alpha shape of the point cloud projected onto the XY plane.
        
        Args:
            save_fig (bool, optional): Whether to save the figure. Defaults to False.
            filename (str, optional): Filename for saving the figure. Defaults to "alpha_shape.png".
        """
        if self.alpha_shape is None:
            logger.warning("Alpha shape has not been computed yet.")
            return
        
        alpha_shape_area = self.alpha_shape_area
        plt.figure()
        if isinstance(self.alpha_shape, Polygon):
            plt.fill(*self.alpha_shape.exterior.xy, color='blue', alpha=0.2,
                     label=f'Alpha Shape (Area: {alpha_shape_area:.2f})')
        elif isinstance(self.alpha_shape, MultiPolygon):
            for polygon in self.alpha_shape:
                plt.fill(*polygon.exterior.xy, color='blue', alpha=0.2)
        plt.title(f"Alpha Shape Area: {alpha_shape_area:.2f} square units")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.legend()
        if save_fig:
            plt.savefig(filename)
        plt.show()

    def plot_points_and_alpha_shape(self, save_fig=False, filename="points_and_alpha_shape.png"):
        """
        Plots the original points and the alpha shape together.
        
        Args:
            save_fig (bool, optional): Whether to save the figure. Defaults to False.
            filename (str, optional): Filename for saving the figure. Defaults to "points_and_alpha_shape.png".
        """
        if self.alpha_shape is None:
            logger.warning("Alpha shape has not been computed yet.")
            return
        
        xy_points = self.points[:, :2]
        alpha_shape_area = self.alpha_shape_area
        plt.figure()
        plt.plot(xy_points[:, 0], xy_points[:, 1], 'o', label='Points', markersize=3)
        if isinstance(self.alpha_shape, Polygon):
            plt.fill(*self.alpha_shape.exterior.xy, color='blue', alpha=0.2,
                     label=f'Alpha Shape (Area: {alpha_shape_area:.2f})')
        elif isinstance(self.alpha_shape, MultiPolygon):
            for polygon in self.alpha_shape:
                plt.fill(*polygon.exterior.xy, color='blue', alpha=0.2)
        plt.title(f"Alpha Shape Area: {alpha_shape_area:.2f} square units")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.legend()
        if save_fig:
            plt.savefig(filename)
        plt.show()
