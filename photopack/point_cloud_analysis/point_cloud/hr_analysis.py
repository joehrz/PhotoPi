# photopack/point_cloud_analysis/point_cloud/hr_analysis.py

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import logging

from photopack.point_cloud_analysis.utils.helpers import cartesian_to_cylindrical
from photopack.point_cloud_analysis.point_cloud.convex_hull import ConvexHullAnalyzer

# Configure logging (optional if already configured in main.py)
logger = logging.getLogger(__name__)


class HRAnalyzer:
    def __init__(self, point_cloud, scale):
        """
        Initializes the HRAnalyzer with the point cloud and scaling factor.

        Args:
            point_cloud (open3d.geometry.PointCloud): The processed point cloud.
            scale (float): Scaling factor based on turntable diameter.
        """
        self.point_cloud = point_cloud
        self.points = np.asarray(point_cloud.points)
        self.scale = scale

        # Attributes to store analysis results
        self.height_density = None
        self.height_mean_r = None
        self.max_radius = None
        self.canopy_volume = None
        self.hr_ratio_density = None
        self.hr_ratio_mean_r = None

        # Additional attributes for internal computations
        self.z_min_density = None
        self.z_min_mean_r = None
        self.z_max = None
        self.z_slice_centers_density = None
        self.densities_smooth = None
        self.z_slice_centers_mean_r = None
        self.mean_r_values = None
        self.r_values = None

    def compute_height(self, z_slice_height=0.01):
        """
        Computes the plant height using two methods: density-based and mean radial distance.

        Args:
            z_slice_height (float, optional): The height of each z-slice for analysis. Defaults to 0.01.
        """
        # Detect z_min using density-based method
        (self.z_min_density,
         self.z_slice_centers_density,
         self.densities_smooth) = self._detect_main_stem_bottom(z_slice_height)

        # Detect z_min using mean radial distance method
        (self.z_min_mean_r,
         self.z_slice_centers_mean_r,
         self.mean_r_values) = self._detect_main_stem_bottom_mean_r(z_slice_height)

        # Calculate z_max
        self.z_max = np.max(self.points[:, 2])

        # Calculate plant heights
        self.height_density = (self.z_max - self.z_min_density) * self.scale
        self.height_mean_r = (self.z_max - self.z_min_mean_r) * self.scale

    def compute_radius(self):
        """
        Computes the maximum radius of the plant.
        """
        x = self.points[:, 0]
        y = self.points[:, 1]
        self.r_values = np.sqrt(x**2 + y**2)
        self.max_radius = np.max(self.r_values) * self.scale

    def compute_volume(self):
        """
        Computes the canopy volume using the convex hull of the point cloud.
        """
        convex_hull_analyzer = ConvexHullAnalyzer(self.point_cloud)
        self.canopy_volume = convex_hull_analyzer.compute_convex_hull_volume(scale=self.scale)

    def compute_ratio(self):
        """
        Computes the Height-to-Radius ratio using both height estimation methods.
        """
        if self.max_radius != 0:
            self.hr_ratio_density = self.height_density / self.max_radius
            self.hr_ratio_mean_r = self.height_mean_r / self.max_radius
        else:
            self.hr_ratio_density = None
            self.hr_ratio_mean_r = None

    def analyze_hr(self):
        """
        Runs the complete H/R analysis, computing height, radius, volume, and ratios.
        """
        self.compute_height()
        self.compute_radius()
        self.compute_volume()
        self.compute_ratio()

        # Log analysis results
        logger.info(f"Plant Height (Density Method): {self.height_density:.2f} cm")
        logger.info(f"Plant Height (Mean R Method): {self.height_mean_r:.2f} cm")
        logger.info(f"Max Radius: {self.max_radius:.2f} cm")
        logger.info(f"Canopy Volume: {self.canopy_volume:.2f} cm³")
        logger.info(f"Height-to-Radius Ratio (Density Method): {self.hr_ratio_density:.2f}")
        logger.info(f"Height-to-Radius Ratio (Mean R Method): {self.hr_ratio_mean_r:.2f}")

    def plot_density_vs_z(self):
        """
        Plots the point density near the origin versus Z using the density-based method.
        """
        plt.figure(figsize=(10, 6))
        plt.plot(self.z_slice_centers_density, self.densities_smooth, label='Point Density Near Origin')
        plt.axvline(self.z_min_density, color='red', linestyle='dashed', linewidth=2,
                    label=f'Estimated Z Min (Density Method): {self.z_min_density:.2f}')
        plt.xlabel('Z-Value (Height)')
        plt.ylabel('Point Density Near Origin')
        plt.title('Density of Points Near Origin vs Z (Density-Based Method)')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_mean_r_vs_z(self):
        """
        Plots the mean radial distance versus Z using the mean radial distance method.
        """
        plt.figure(figsize=(10, 6))
        plt.plot(self.z_slice_centers_mean_r, self.mean_r_values, label='Mean Radial Distance')
        plt.axvline(self.z_min_mean_r, color='green', linestyle='dashed', linewidth=2,
                    label=f'Estimated Z Min (Mean R Method): {self.z_min_mean_r:.2f}')
        plt.xlabel('Z-Value (Height)')
        plt.ylabel('Mean Radial Distance')
        plt.title('Mean Radial Distance vs Z (Mean Radial Distance Method)')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_height_vs_radius(self):
        """
        Plots Height vs Radius of the plant with Z Min estimates from both methods.
        """
        z_cyl = self.points[:, 2]
        plt.figure(figsize=(10, 6))
        plt.scatter(self.r_values, z_cyl, alpha=0.1, s=1, c='blue', label='Point Cloud')
        plt.axhline(self.z_min_density, color='red', linestyle='dashed', linewidth=2,
                    label=f'Z Min (Density Method): {self.z_min_density:.2f}')
        plt.axhline(self.z_min_mean_r, color='green', linestyle='dashed', linewidth=2,
                    label=f'Z Min (Mean R Method): {self.z_min_mean_r:.2f}')
        plt.xlabel('Radius')
        plt.ylabel('Height (Z)')
        plt.title('Height vs Radius of Plant with Z Min Estimates')
        plt.legend()
        plt.grid(True)
        plt.show()

    # Internal helper methods
    def _detect_main_stem_bottom(self, z_slice_height=0.01):
        """
        Detects the bottom of the main stem using a density-based method.

        Args:
            z_slice_height (float, optional): The height of each z-slice for analysis. Defaults to 0.01.

        Returns:
            tuple: Estimated Z Min, Z slice centers, and smoothed densities.
        """
        x = self.points[:, 0]
        y = self.points[:, 1]
        z = self.points[:, 2]

        # Calculate radial distances
        r = np.sqrt(x**2 + y**2)

        # Adaptive r_threshold
        r_threshold = np.percentile(r, 5)

        # Define z-slices
        z_min = np.min(z)
        z_max = np.max(z)
        z_slices = np.arange(z_min, z_max, z_slice_height)

        densities = []
        z_slice_centers = []

        for z_start in z_slices:
            z_end = z_start + z_slice_height
            # Select points within the current z-slice
            mask_z = (z >= z_start) & (z < z_end)
            x_slice = x[mask_z]
            y_slice = y[mask_z]
            # Calculate radial distance from origin
            r_slice = np.sqrt(x_slice**2 + y_slice**2)
            # Count points within r_threshold
            num_points = np.sum(r_slice <= r_threshold)
            densities.append(num_points)
            z_slice_centers.append(z_start + z_slice_height / 2)

        densities = np.array(densities)
        z_slice_centers = np.array(z_slice_centers)

        # Smooth the density curve
        from scipy.ndimage import gaussian_filter1d
        sigma = len(densities) * 0.02
        densities_smooth = gaussian_filter1d(densities, sigma=sigma)

        # Compute the first derivative
        density_derivative = np.diff(densities_smooth) / z_slice_height

        # Adaptive derivative threshold
        mean_derivative = np.mean(density_derivative)
        std_derivative = np.std(density_derivative)
        derivative_threshold = mean_derivative + 2 * std_derivative

        indices = np.where(density_derivative > derivative_threshold)[0]

        if len(indices) > 0:
            z_min_index = indices[0]
            z_min_estimated = z_slice_centers[z_min_index]
        else:
            # Default to a percentile
            z_min_estimated = np.percentile(z, 5)

        return z_min_estimated, z_slice_centers, densities_smooth

    def _detect_main_stem_bottom_mean_r(self, z_slice_height=0.01):
        """
        Detects the bottom of the main stem using the mean radial distance method.

        Args:
            z_slice_height (float, optional): The height of each z-slice for analysis. Defaults to 0.01.

        Returns:
            tuple: Estimated Z Min, Z slice centers, and mean radial values.
        """
        x = self.points[:, 0]
        y = self.points[:, 1]
        z = self.points[:, 2]

        z_min = np.min(z)
        z_max = np.max(z)
        z_slices = np.arange(z_min, z_max, z_slice_height)

        mean_r_values = []
        z_slice_centers = []

        for z_start in z_slices:
            z_end = z_start + z_slice_height
            mask_z = (z >= z_start) & (z < z_end)
            x_slice = x[mask_z]
            y_slice = y[mask_z]
            r_slice = np.sqrt(x_slice**2 + y_slice**2)
            if len(r_slice) > 0:
                mean_r = np.mean(r_slice)
                mean_r_values.append(mean_r)
                z_slice_centers.append(z_start + z_slice_height / 2)

        mean_r_values = np.array(mean_r_values)
        z_slice_centers = np.array(z_slice_centers)

        # Compute relative changes
        mean_r_diff = np.diff(mean_r_values)
        relative_changes = mean_r_diff / mean_r_values[:-1]

        # Adaptive threshold for significant drop
        mean_change = np.mean(relative_changes)
        std_change = np.std(relative_changes)
        threshold_drop = mean_change - 1 * std_change  # Adjust the multiplier as needed

        indices = np.where(relative_changes < threshold_drop)[0]

        if len(indices) > 0:
            z_min_index = indices[0]
            z_min_estimated = z_slice_centers[z_min_index]
        else:
            z_min_estimated = np.percentile(z, 5)

        return z_min_estimated, z_slice_centers, mean_r_values


