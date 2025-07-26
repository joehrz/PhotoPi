import numpy as np
import open3d as o3d
import logging
import copy
import matplotlib.pyplot as plt

from point_cloud.convex_hull import ConvexHullAnalyzer
from point_cloud.main_stem_segmentation import MainStemSegmentation

logger = logging.getLogger(__name__)


class HRAnalyzer:
    def __init__(self, point_cloud, scale):
        """
        Initializes the HRAnalyzer with the point cloud and scaling factor.

        Args:
            point_cloud (open3d.geometry.PointCloud): The processed point cloud.
            scale (float): Scaling factor based on turntable diameter or known reference.
        """
        self.original_pcd = point_cloud
        self.pcd = copy.deepcopy(point_cloud)
        self.scale = scale

        # Computed results
        self.height = None
        self.max_radius = None
        self.height_unscaled = None
        self.max_radius_unscaled = None
        self.canopy_volume = None
        self.hr_ratio = None
        self.z_max = None  # For visualization

        # Debug/visualization data
        self.bottom_cluster_pts = None         # All points in the bottom cluster
        self.bottom_cluster_lowest_pts = None  # The subset with the absolute min z in that cluster

    def analyze_hr_with_mainstem(
        self,
        alpha=1.0,
        beta=0.5,
        raindrop_alpha=1.0,
        raindrop_beta=1.0,
        gamma=10.0,
        delta=5.0,
        use_trunk_axis=True,
        debug=True
    ):
        """
        1) Use MainStemSegmentation to run the full pipeline (slicing, adjacency, bridging, etc.).
        2) Identify the base node from seg.base_node along the trunk path.
        3) Retrieve the actual cluster points in that base node’s slice.
        4) Shift the entire point cloud by that bottom cluster's centroid so that 
           the cluster is 'balanced' around the origin.
        5) Compute height, radius, volume, and H/R ratio.

        After recentering, the physically lowest point of the cluster may have negative z 
        (if it lies below its centroid), but the cluster’s average is at the origin.
        """
        logger.info("[analyze_hr_with_mainstem] Running MainStemSegmentation pipeline.")
        seg = MainStemSegmentation(self.pcd)

        # 1) Run the new pipeline (slicing, bridging, trunk extraction, etc.)
        seg.run_full_pipeline(
            alpha=alpha,
            beta=beta,
            raindrop_alpha=raindrop_alpha,
            raindrop_beta=raindrop_beta,
            gamma=gamma,
            delta=delta,
            use_trunk_axis=use_trunk_axis,
            debug=debug
        )

        # 2) Identify base node from the trunk path
        base_node = seg.base_node
        trunk_path = seg.trunk_path
        if base_node is None or not trunk_path:
            logger.warning("No base_node / trunk_path found; cannot compute HR metrics.")
            return

        logger.info(f"[analyze_hr_with_mainstem] trunk_path length={len(trunk_path)}, base_node={base_node}")

        # 3) Retrieve the cluster points belonging to that base node
        #    The new segmentation pipeline keeps:
        #      - self.aggregated_centroids_map[node_id] -> (slice_i, cluster_j)
        #      - self.slice_results[slice_i][cluster_j]['points']
        slice_i, cluster_j = seg.aggregated_centroids_map[base_node]
        self.bottom_cluster_pts = seg.slice_results[slice_i][cluster_j]['points']

        # (Optional) identify the absolute lowest points in that cluster
        if len(self.bottom_cluster_pts) > 0:
            z_min_cluster = np.min(self.bottom_cluster_pts[:, 2])
            mask_lowest = (self.bottom_cluster_pts[:, 2] == z_min_cluster)
            self.bottom_cluster_lowest_pts = self.bottom_cluster_pts[mask_lowest]
            logger.info(f"[analyze_hr_with_mainstem] bottom cluster => total={len(self.bottom_cluster_pts)}, "
                        f"lowest subset={len(self.bottom_cluster_lowest_pts)}, z_min={z_min_cluster:.3f}")
        else:
            logger.warning("Bottom cluster is empty; skipping re-center.")
            return

        # 4) Shift the entire point cloud by the centroid of this bottom cluster
        self._recenter_bottom_cluster_by_centroid()

        # 5) Compute height, radius, volume, H/R ratio
        self._compute_height()
        self._compute_radius()
        self._compute_volume()
        self._compute_hr_ratio()

        logger.info(
            f"[analyze_hr_with_mainstem] Final => "
            f"Height={self.height:.2f} cm, "
            f"Radius={self.max_radius:.2f} cm, "
            f"Volume={self.canopy_volume:.2f} cm³, "
            f"H/R={self.hr_ratio:.2f}"
        )

    def _recenter_bottom_cluster_by_centroid(self):
        """
        Shift the original point cloud by the *centroid* (x,y,z) of bottom_cluster_pts,
        so the entire cluster is 'balanced' around (0,0,0).
        """
        pts = np.asarray(self.pcd.points)
        if len(pts) == 0:
            logger.warning("Point cloud is empty, skipping recenter.")
            return

        if self.bottom_cluster_pts is None or len(self.bottom_cluster_pts) == 0:
            logger.warning("[_recenter_bottom_cluster_by_centroid] No bottom cluster data; skipping.")
            return

        # 1) compute the centroid of bottom_cluster_pts
        centroid_3d = np.mean(self.bottom_cluster_pts, axis=0)
        logger.info(f"[_recenter_bottom_cluster_by_centroid] Shifting by centroid={centroid_3d}")

        # 2) shift the entire cloud
        pts -= centroid_3d
        self.pcd.points = o3d.utility.Vector3dVector(pts)

        logger.info("[_recenter_bottom_cluster_by_centroid] Done => bottom cluster centroid at (0,0,0).")

    # ----------------------------------------------------------
    # 2. Compute Basic Metrics (Height, Radius, Volume)
    # ----------------------------------------------------------
    def _compute_height(self):
        """
        After recentering, height is max(Z) * scale.
        """
        pts = np.asarray(self.pcd.points)
        if len(pts) == 0:
            self.height = 0.0
            return
        self.z_max = np.max(pts[:, 2])
        self.height = self.z_max * self.scale
        self.height_unscaled = self.z_max

    def _compute_radius(self):
        """
        Max radius from origin in xy-plane, then times scale.
        """
        pts = np.asarray(self.pcd.points)
        if len(pts) == 0:
            self.max_radius = 0.0
            return
        r_vals = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        self.max_radius = np.max(r_vals) * self.scale
        self.max_radius_unscaled = np.max(r_vals)

    def _compute_volume(self):
        """
        Use the convex hull to compute volume of the recentered point cloud.
        """
        if len(self.pcd.points) == 0:
            self.canopy_volume = 0.0
            return
        hull_analyzer = ConvexHullAnalyzer(self.pcd)
        self.canopy_volume = hull_analyzer.compute_convex_hull_volume(scale=self.scale)

    def _compute_hr_ratio(self):
        """
        H/R ratio = self.height / self.max_radius
        """
        if not self.max_radius or self.max_radius == 0:
            self.hr_ratio = None
        else:
            self.hr_ratio = self.height / self.max_radius

    # ----------------------------------------------------------
    # Visualization
    # ----------------------------------------------------------
    def visualize_with_open3d(self):
        """
        Show the recentered point cloud + base (0,0,0) + top (0,0,z_max)
        plus highlight the bottom cluster or lowest subset if available.
        """
        geoms = []

        # 1) recentered point cloud
        geoms.append(self.pcd)

        # 2) Add coordinate axes
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        geoms.append(coord_frame)

        # 3) Possibly place spheres for base & top 
        if self.z_max is None:
            pts = np.asarray(self.pcd.points)
            self.z_max = np.max(pts[:, 2]) if len(pts) else 0.0

        sphere_base = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
        sphere_base.paint_uniform_color([1, 0, 0])  # red
        sphere_base.translate([0, 0, 0])

        sphere_top = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
        sphere_top.paint_uniform_color([0, 1, 0])  # green
        sphere_top.translate([0, 0, self.z_max])
        geoms.extend([sphere_base, sphere_top])

        # # 4) line from base to top
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector([
            [0, 0, 0],
            [0, 0, self.z_max]
        ])
        line_set.lines = o3d.utility.Vector2iVector([[0, 1]])
        line_set.colors = o3d.utility.Vector3dVector([[0, 0, 0]])  # black
        geoms.append(line_set)

        # 5) display
        o3d.visualization.draw_geometries(geoms, window_name="HR Analysis Visualization")

    def plot_height_vs_radius(self):
        """
        Example: Plot Z vs. radius, for a quick 2D check.
        """
        pts = np.asarray(self.pcd.points)
        if len(pts) == 0:
            logger.warning("No points available for plotting.")
            return
        r_vals = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        z_cyl  = pts[:, 2]

        plt.figure(figsize=(10, 6))
        plt.scatter(r_vals, z_cyl, alpha=0.1, s=1, c='blue', label='Point Cloud')

        # reference line at z=0
        plt.axhline(0.0, color='red', linestyle='dashed', linewidth=2, label='Z=0')

        plt.xlabel('Radius')
        plt.ylabel('Z')
        plt.title('Height vs Radius (Recentered at Bottom Cluster Centroid)')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_radius_vs_theta(self):
        """
        Plot a polar plot of theta and radius of the points.
        """
        # Increase font sizes globally
        plt.rcParams.update({'font.size': 24,
                             'axes.labelsize': 24,
                             'axes.titlesize': 18,
                             'xtick.labelsize': 24,
                             'ytick.labelsize': 24,
                             'legend.fontsize': 24})
    
        pts = np.asarray(self.pcd.points)
        if len(pts) == 0:
            logger.warning("No points available for plotting.")
            return
        # Compute radius and theta
        r_vals = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        theta_vals = np.arctan2(pts[:, 1], pts[:, 0])
        max_r = np.max(r_vals)
        max_r_idx = np.argmax(r_vals)  # Get the index of the maximum radius
        max_r_theta = theta_vals[max_r_idx]  # Get theta at that index
    
        # Create polar plot
        plt.figure(figsize=(10, 6))
        ax = plt.subplot(111, projection='polar')
        ax.scatter(theta_vals, r_vals, alpha=0.1, s=1, c='blue', label='Point Cloud')
        ax.scatter(max_r_theta, max_r, alpha=1, s=150, c='red', label='Max R')
    
        # ax.set_title('Polar Plot of Radius vs Theta', fontsize=18)
        ax.tick_params(axis='both', which='major', labelsize=20)
    
        plt.legend(loc='lower right', fontsize=20)
        plt.tight_layout()  # Adjusts the plot to make room for larger font sizes
        plt.show()