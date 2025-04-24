# photopack/point_cloud_analysis/point_cloud/processing.py

import numpy as np
import open3d as o3d
import logging
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import copy

# Configure logging
logger = logging.getLogger(__name__)


class PointCloudProcessor:
    def __init__(self, point_cloud):
        """
        Initializes the PointCloudProcessor with the point cloud.

        Args:
            point_cloud (open3d.geometry.PointCloud): The point cloud to process.
        """
        self.original_point_cloud = point_cloud
        self.point_cloud = copy.deepcopy(point_cloud)  # Use deepcopy instead of clone()
        self.points = np.asarray(self.point_cloud.points)
        self.colors = np.asarray(self.point_cloud.colors)
        self.normals = None
        if self.point_cloud.has_normals():
            self.normals = np.asarray(self.point_cloud.normals)
        self.processed_point_cloud = None

        # Internal attributes
        self.centroid = None
        self.plane_model = None
        self.plane_inliers = None
        self.plane_normal = None
        self.ring_center = None
        self.ring_radius = None
        self.diameter = None
        self.turntable_pcd = None
        self.ring_points = None

    def center_point_cloud(self):
        """
        Centers the point cloud at the origin.
        """
        self.centroid = np.mean(self.points, axis=0)
        self.points -= self.centroid
        logger.info("Point cloud centered at the origin.")


    def filter_non_points(self, lower_threshold, upper_threshold):
        """
        Filters out points that do not have color values within the specified lower and upper thresholds.

        Args:
            lower_threshold (np.ndarray): Lower RGB color threshold.
            upper_threshold (np.ndarray): Upper RGB color threshold.
        """
        # Identify points outside the specified color thresholds
        outside_threshold_indices = np.all(
            (self.colors < lower_threshold) | (self.colors > upper_threshold), axis=1
        )
        # Keep points within the thresholds
        in_threshold_indices = outside_threshold_indices
        filtered_points = self.points[in_threshold_indices]
        filtered_colors = self.colors[in_threshold_indices]

        # Update the point cloud to contain only the filtered points
        self.point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        self.point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)
        self.points = np.asarray(self.point_cloud.points)
        self.colors = np.asarray(self.point_cloud.colors)
        logger.info(f"Filtered points based on color thresholds. Remaining points: {len(filtered_points)}")

    def segment_plane(self, distance_threshold=0.01, ransac_n=3, num_iterations=1000):
        """
        Segments the dominant plane in the point cloud (turntable plane).

        Args:
            distance_threshold (float): Maximum distance a point can be from the plane to be considered an inlier.
            ransac_n (int): Number of initial points to estimate a plane.
            num_iterations (int): Number of RANSAC iterations.
        """
        plane_model, inliers = self.point_cloud.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        self.plane_model = plane_model
        self.plane_inliers = inliers
        self.plane_cloud = self.point_cloud.select_by_index(inliers)
        self.outlier_cloud = self.point_cloud.select_by_index(inliers, invert=True)


        logger.info(f"Plane model estimated: {plane_model}")


    def align_to_z_axis(self):
        """
        Rotates the point cloud so that the plane normal aligns with the Z-axis.
        """
        normal_vector = np.array(self.plane_model[:3])
        direction_vector = np.mean(np.asarray(self.outlier_cloud.points), axis=0) - np.mean(np.asarray(self.plane_cloud.points), axis=0)

        if np.dot(normal_vector, direction_vector) < 0:
            normal_vector = -normal_vector

        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(normal_vector, z_axis)
        if np.linalg.norm(rotation_axis) < 1e-6:
            # The normal is already aligned with z-axis
            rotation_matrix = np.eye(3)
        else:
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            angle = np.arccos(np.clip(np.dot(normal_vector, z_axis), -1.0, 1.0))
            rotation_matrix = R.from_rotvec(angle * rotation_axis).as_matrix()
        #np.asarray(outlier_cloud.points)
        self.points[:] = self.points @ rotation_matrix.T
        logger.info("Point cloud aligned with Z-axis.")

    def translate_to_origin(self):
        """
        Translates the point cloud so that the plane aligns with z=0.
        """
        d = self.plane_model[3]
        translation_z = abs(d)
        self.points[:, 2] += translation_z
        logger.info(f"Point cloud translated along Z by {translation_z} to align plane with z=0.")

    def filter_points_by_z_interval(self, z_min=-0.4, z_max=0):
        """
        Filters points within a Z-value interval.

        Args:
            z_min (float): Minimum Z value.
            z_max (float): Maximum Z value.
        """
        mask = (self.points[:, 2] > z_min) & (self.points[:, 2] < z_max)
        self.points_z_filtered = self.points[mask]
        self.colors_z_filtered = self.colors[mask]
        logger.info(f"Filtered points by Z interval ({z_min}, {z_max}). Remaining points: {len(self.points_z_filtered)}")

    def segment_ring_points(self, num_bins=6000, tolerance=0.2):
        """
        Segments ring points from the point cloud based on radial distance density.

        Args:
            num_bins (int): Number of bins for the histogram.
            tolerance (float): Tolerance around the peak radius to select ring points.
        """
        r = np.sqrt(self.points_z_filtered[:, 0] ** 2 + self.points_z_filtered[:, 1] ** 2)
        hist, bin_edges = np.histogram(r, bins=num_bins)
        max_bin_index = np.argmax(hist)
        peak_radius = (bin_edges[max_bin_index] + bin_edges[max_bin_index + 1]) / 2
        ring_indices = np.where((r >= peak_radius - tolerance) & (r <= peak_radius + tolerance))[0]
        self.ring_points = self.points_z_filtered[ring_indices]
        logger.info(f"Segmented ring points. Number of ring points: {len(self.ring_points)}")

    def refine_ring_points_with_dbscan(self, eps=0.1, min_samples=5):
        """
        Refines ring points using DBSCAN clustering.

        Args:
            eps (float): Maximum distance between two samples for them to be considered as in the same neighborhood.
            min_samples (int): Minimum number of samples in a neighborhood for a point to be a core point.
        """
        if len(self.ring_points) == 0:
            logger.warning("No ring points to refine with DBSCAN.")
            return
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(self.ring_points)
        labels = clustering.labels_
        unique_labels, counts = np.unique(labels, return_counts=True)
        if len(unique_labels) == 0:
            logger.warning("DBSCAN did not find any clusters in ring points.")
            return
        largest_cluster_label = unique_labels[np.argmax(counts)]
        self.ring_points = self.ring_points[labels == largest_cluster_label]
        logger.info(f"Refined ring points with DBSCAN. Number of ring points: {len(self.ring_points)}")

    def estimate_ring_center_and_radius(self):
        """
        Estimates the center and radius of the ring.
        """
        if len(self.ring_points) == 0:
            logger.warning("No ring points to estimate center and radius.")
            return
        projected_points = self.ring_points[:, :2]
        center_xy = np.mean(projected_points, axis=0)
        distances = np.linalg.norm(projected_points - center_xy, axis=1)
        estimated_radius = np.mean(distances)
        self.ring_center = np.array([center_xy[0], center_xy[1], np.mean(self.ring_points[:, 2])])
        self.ring_radius = estimated_radius
        self.diameter = estimated_radius * 2
        logger.info(f"Estimated ring center: {self.ring_center}, radius: {self.ring_radius}, diameter: {self.diameter}")

    def adjust_point_cloud_center(self):
        """
        Adjusts the point cloud so that the ring center is at the origin.
        """
        self.points[:, 0] -= self.ring_center[0]
        self.points[:, 1] -= self.ring_center[1]
        logger.info("Adjusted point cloud center to align ring center with origin.")

    def process(self, outpath):
        """
        Starts the processing steps.
        """
        try:
            self.center_point_cloud()
            # Define color thresholds for plant filtering
            lower_threshold = np.array([0.03529412, 0.3254902, 0.8])
            upper_threshold = np.array([0.09411765, 0.41176471, 0.89411765])
            self.filter_non_points(lower_threshold, upper_threshold)
            self.segment_plane()
            self.align_to_z_axis()
            self.translate_to_origin()
            self.filter_points_by_z_interval(z_min=-0.4, z_max=0)
            self.segment_ring_points()
            self.refine_ring_points_with_dbscan()
            self.estimate_ring_center_and_radius()
            if self.ring_center is not None:
                self.adjust_point_cloud_center()
            else:
                logger.warning("Ring center not estimated. Skipping center adjustment.")
            self.processed_point_cloud = self.point_cloud
            self.visualize()
            self.save_processed_point_cloud(outpath)
            logger.info("Point cloud processing completed.")
        except Exception as e:
            logger.error(f"Error during point cloud processing: {e}")

    def save_processed_point_cloud(self, filename):
        """
        Saves the processed point cloud to a file.

        Args:
            filename (str): Path to save the processed point cloud.
        """
        if self.processed_point_cloud is not None:
            o3d.io.write_point_cloud(filename, self.processed_point_cloud)
            logger.info(f"Processed point cloud saved to {filename}")
        else:
            logger.error("Processed point cloud is None. Cannot save.")

    def visualize(self):
        """
        Visualizes the processed point cloud along with the coordinate frame.
        """
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        if self.processed_point_cloud is not None:
            o3d.visualization.draw_geometries([self.processed_point_cloud, coordinate_frame])
        else:
            logger.error("Processed point cloud is None. Cannot visualize.")


