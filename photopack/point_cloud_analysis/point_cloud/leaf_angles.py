import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import logging
import networkx as nx
import math

from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.decomposition import PCA
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.interpolate import splprep, splev
import hdbscan

# This import is necessary to indicate the expected type for the segmentation instance.
from photopack.point_cloud_analysis.point_cloud.main_stem_segmentation import MainStemSegmentation

logger = logging.getLogger(__name__)


# --- Utility Functions ---
def perform_linear_regression(points):
    """
    Perform linear regression on a set of 3D points.
    Here, the z-coordinate is used as the independent variable and x,y are predicted.
    Returns predicted x, y, and uniformly spaced z values.
    """
    if len(points) < 2:
        return points[:, 0], points[:, 1], points[:, 2]

    X = points[:, 2].reshape(-1, 1)
    reg_x = LinearRegression().fit(X, points[:, 0])
    reg_y = LinearRegression().fit(X, points[:, 1])
    z_vals = np.linspace(np.min(X), np.max(X), 100).reshape(-1, 1)
    x_vals = reg_x.predict(z_vals)
    y_vals = reg_y.predict(z_vals)
    return x_vals, y_vals, z_vals.flatten()


def calculate_angle_between_vectors(v1, v2):
    """
    Calculate and return the angle in degrees between two 3D vectors.
    """
    dot = np.dot(v1, v2)
    mag1 = np.linalg.norm(v1)
    mag2 = np.linalg.norm(v2)
    if mag1 == 0 or mag2 == 0:
        return None
    cos = np.clip(dot / (mag1 * mag2), -1, 1)
    return math.degrees(np.arccos(cos))


def extend_vector(start_point, direction_vector, scale=1.5):
    """
    Extend a vector from a starting point by a given scale.
    """
    return start_point + scale * direction_vector


def create_arc_points(center, start_point, end_point, num_points=20):
    """
    Create a set of points forming an arc from start_point to end_point around the center.
    """
    v1 = start_point - center
    v2 = end_point - center
    norm1, norm2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if norm1 < 1e-9 or norm2 < 1e-9:
        return np.vstack((center, start_point, end_point))

    v1n, v2n = v1 / norm1, v2 / norm2
    dot = np.clip(np.dot(v1n, v2n), -1, 1)
    angle = np.arccos(dot)
    arc_points = []
    axis = np.cross(v1n, v2n)
    axis_len = np.linalg.norm(axis)
    if axis_len < 1e-9:
        return np.vstack((center, start_point, end_point))
    axis /= axis_len

    for t in np.linspace(0, 1, num_points):
        theta = t * angle
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        R = np.eye(3)*np.cos(theta) + np.sin(theta)*K + (1 - np.cos(theta)) * np.outer(axis, axis)
        arc_points.append(center + R @ v1)
    return np.array(arc_points)


# --- Main Leaf Angle Analyzer Class ---
class LeafAngleAnalyzer:
    """
    LeafAngleAnalyzer computes leaf angles from segmented plant point cloud data.
    
    This class is designed to work with an instance of MainStemSegmentation that has already 
    processed the point cloud. It expects that the segmentation instance contains the following:
      - all_main_stem_points_up: List of 3D point arrays (upward main stem points per branch)
      - all_main_stem_points_down: List of 3D point arrays (downward main stem points per branch)
      - all_leaf_points: List of 3D point arrays for each branch (leaf regions)
    
    The leaf angle is computed by fitting linear regression models (using z as the independent variable)
    to both the main stem points and the leaf points in each branch region. The angle between the 
    resulting direction vectors is then calculated.
    """
    def __init__(self, segmentation: MainStemSegmentation):
        """
        Initialize the LeafAngleAnalyzer.
        
        Parameters:
            segmentation (MainStemSegmentation): A fully processed segmentation instance containing
                                                  the necessary segmented point sets.
        """
        self.segmentation = segmentation
        self.angles = []  # List to store computed leaf angles (in degrees)

    def compute_leaf_angles(self):
        """
        Compute leaf angles for each branch region.
        
        For each branch (i.e. for each set of extracted main stem and leaf points),
        the method performs the following:
          - Combines the upward and downward main stem points into one array.
          - Fits linear regression models to the main stem points and leaf points (using z as the independent variable).
          - Derives direction vectors from the endpoints of the regression lines.
          - Calculates the angle (in degrees) between the two direction vectors.
        
        Returns:
            list: A list of computed leaf angles. For branch regions with insufficient points, None is appended.
        """
        for i in range(len(self.segmentation.all_main_stem_points_up)):
            # Combine up and down main stem points for the branch
            stem_pts = np.vstack((self.segmentation.all_main_stem_points_up[i],
                                  self.segmentation.all_main_stem_points_down[i]))
            leaf_pts = self.segmentation.all_leaf_points[i]

            # Check if there are enough points for regression
            if len(stem_pts) < 2 or len(leaf_pts) < 2:
                logger.warning(f"Branch {i+1} => not enough points for regression. Skipping.")
                self.angles.append(None)
                continue

            x_st, y_st, z_st = perform_linear_regression(stem_pts)
            x_lf, y_lf, z_lf = perform_linear_regression(leaf_pts)

            # Compute direction vectors from the regression endpoints
            stem_vec = np.array([x_st[-1], y_st[-1], z_st[-1]]) - np.array([x_st[0], y_st[0], z_st[0]])
            leaf_vec = np.array([x_lf[-1], y_lf[-1], z_lf[-1]]) - np.array([x_lf[0], y_lf[0], z_lf[0]])

            angle_deg = calculate_angle_between_vectors(stem_vec, leaf_vec)
            self.angles.append(angle_deg)
            logger.info(f"Branch-off {i+1} => Leaf angle = {angle_deg:.2f} deg")
        return self.angles

    # ------------------- Visualization Methods ------------------- #
    def visualize_results(self):
        """
        Visualize local geometry for each branch region by displaying:
          - Main stem points in red.
          - Leaf points in green.
          - Lines from the branch point (start of regression) to the extended regression endpoints.
          - An arc representing the computed angle.
          - The leaf regression line (in green).
        """
        logger.info("visualize_results => Displaying partial geometry (stem/leaf arcs).")
        geoms = []
        for i in range(len(self.segmentation.all_main_stem_points_up)):
            stem_pts = np.vstack((self.segmentation.all_main_stem_points_up[i],
                                  self.segmentation.all_main_stem_points_down[i]))
            leaf_pts = self.segmentation.all_leaf_points[i]
            if len(stem_pts) < 2 or len(leaf_pts) < 2:
                continue

            x_stem, y_stem, z_stem = perform_linear_regression(stem_pts)
            x_leaf, y_leaf, z_leaf = perform_linear_regression(leaf_pts)

            stem_vec = np.array([x_stem[-1], y_stem[-1], z_stem[-1]]) - np.array([x_stem[0], y_stem[0], z_stem[0]])
            leaf_vec = np.array([x_leaf[-1], y_leaf[-1], z_leaf[-1]]) - np.array([x_leaf[0], y_leaf[0], z_leaf[0]])

            branch_pt = np.array([x_stem[0], y_stem[0], z_stem[0]])
            ext_stem = extend_vector(branch_pt, stem_vec)
            ext_leaf = extend_vector(branch_pt, leaf_vec)
            arc_pts = create_arc_points(branch_pt, ext_stem, ext_leaf)

            # Create and color the point clouds
            pcd_stem = o3d.geometry.PointCloud()
            pcd_stem.points = o3d.utility.Vector3dVector(stem_pts)
            pcd_stem.paint_uniform_color([1, 0, 0])
            geoms.append(pcd_stem)

            pcd_leaf = o3d.geometry.PointCloud()
            pcd_leaf.points = o3d.utility.Vector3dVector(leaf_pts)
            pcd_leaf.paint_uniform_color([0, 1, 0])
            geoms.append(pcd_leaf)

            # Draw lines from branch_pt to the extended endpoints
            arr_pts = np.vstack([
                branch_pt, ext_stem,
                np.array([x_leaf[0], y_leaf[0], z_leaf[0]]), ext_leaf
            ])
            lines_idx = [[0, 1], [2, 3]]
            ls = o3d.geometry.LineSet()
            ls.points = o3d.utility.Vector3dVector(arr_pts)
            ls.lines = o3d.utility.Vector2iVector(lines_idx)
            ls.colors = o3d.utility.Vector3dVector([[0, 0, 1], [0, 0, 1]])
            geoms.append(ls)

            # Draw arc (in pink) representing the angle
            arc_ls = o3d.geometry.LineSet()
            arc_ls.points = o3d.utility.Vector3dVector(arc_pts)
            arc_lines = [[kk, kk+1] for kk in range(len(arc_pts)-1)]
            arc_ls.lines = o3d.utility.Vector2iVector(arc_lines)
            arc_ls.colors = o3d.utility.Vector3dVector([[1, 0, 1] for _ in arc_lines])
            geoms.append(arc_ls)

            # Draw the leaf regression line (in green)
            leaf_reg_pts = np.column_stack((x_leaf, y_leaf, z_leaf))
            leaf_reg_lines = [[kk, kk+1] for kk in range(len(leaf_reg_pts)-1)]
            leaf_reg_ls = o3d.geometry.LineSet()
            leaf_reg_ls.points = o3d.utility.Vector3dVector(leaf_reg_pts)
            leaf_reg_ls.lines = o3d.utility.Vector2iVector(leaf_reg_lines)
            leaf_reg_ls.colors = o3d.utility.Vector3dVector([[0, 1, 0] for _ in leaf_reg_lines])
            geoms.append(leaf_reg_ls)

        if geoms:
            o3d.visualization.draw_geometries(geoms, window_name="Partial Leaf Angles")
        else:
            logger.info("No geometry to visualize (no leaf angles found).")

    def visualize_in_original(self):
        """
        Overlays the leaf angle visualization (arcs and extended regression lines)
        onto the original point cloud.
        """
        logger.info("visualize_in_original => Overlaying leaf angles on original cloud.")
        full_geo = [self.segmentation.pcd]

        for i in range(len(self.segmentation.all_main_stem_points_up)):
            stem_pts = np.vstack((self.segmentation.all_main_stem_points_up[i],
                                  self.segmentation.all_main_stem_points_down[i]))
            leaf_pts = self.segmentation.all_leaf_points[i]
            if len(stem_pts) < 2 or len(leaf_pts) < 2:
                continue

            x_stem, y_stem, z_stem = perform_linear_regression(stem_pts)
            x_leaf, y_leaf, z_leaf = perform_linear_regression(leaf_pts)

            stem_vec = np.array([x_stem[-1], y_stem[-1], z_stem[-1]]) - np.array([x_stem[0], y_stem[0], z_stem[0]])
            leaf_vec = np.array([x_leaf[-1], y_leaf[-1], z_leaf[-1]]) - np.array([x_leaf[0], y_leaf[0], z_leaf[0]])

            branch_pt = np.array([x_stem[0], y_stem[0], z_stem[0]])
            ext_stem = extend_vector(branch_pt, stem_vec)
            ext_leaf = extend_vector(branch_pt, leaf_vec)
            arc_pts = create_arc_points(branch_pt, ext_stem, ext_leaf)

            arr_pts = np.vstack([
                branch_pt, ext_stem,
                np.array([x_leaf[0], y_leaf[0], z_leaf[0]]), ext_leaf
            ])
            lines_idx = [[0, 1], [2, 3]]
            ls = o3d.geometry.LineSet()
            ls.points = o3d.utility.Vector3dVector(arr_pts)
            ls.lines = o3d.utility.Vector2iVector(lines_idx)
            ls.colors = o3d.utility.Vector3dVector([[0, 0, 1], [0, 0, 1]])
            full_geo.append(ls)

            arc_ls = o3d.geometry.LineSet()
            arc_ls.points = o3d.utility.Vector3dVector(arc_pts)
            arc_lines = [[kk, kk+1] for kk in range(len(arc_pts)-1)]
            arc_ls.lines = o3d.utility.Vector2iVector(arc_lines)
            arc_ls.colors = o3d.utility.Vector3dVector([[1, 0, 1] for _ in arc_lines])
            full_geo.append(arc_ls)

        o3d.visualization.draw_geometries(full_geo, window_name="Leaf Angles + Original Cloud")

    def visualize_graph_with_types(self, show_original=False):
        """
        Visualize the centroid graph with nodes color-coded by type:
          'stem' (red), 'branch_off' (blue), 'branch_or_leaf' (green).
        Edges are drawn in black. Optionally, the original point cloud is also displayed.
        """
        logger.info("visualize_graph_with_types => Displaying color-coded centroid graph.")
        geoms = []
        if show_original:
            geoms.append(self.segmentation.point_cloud)

        pos = []
        type_colors = []
        for n in self.graph.nodes():
            p = self.graph.nodes[n]['pos']
            pos.append(p)
            t = self.graph.nodes[n].get('type', 'unknown')
            if t == 'stem':
                type_colors.append([1, 0, 0])
            elif t == 'branch_off':
                type_colors.append([0, 0, 1])
            elif t == 'branch_or_leaf':
                type_colors.append([0, 1, 0])
            else:
                type_colors.append([0.5, 0.5, 0.5])
        pos = np.array(pos)

        pcd_nodes = o3d.geometry.PointCloud()
        pcd_nodes.points = o3d.utility.Vector3dVector(pos)
        pcd_nodes.colors = o3d.utility.Vector3dVector(type_colors)
        geoms.append(pcd_nodes)

        edges = []
        for n in self.graph.nodes():
            for m in self.graph.neighbors(n):
                if m > n:
                    edges.append([n, m])
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(pos)
        ls.lines = o3d.utility.Vector2iVector(np.array(edges))
        ls.colors = o3d.utility.Vector3dVector([[0, 0, 0]] * len(edges))
        geoms.append(ls)

        o3d.visualization.draw_geometries(geoms, window_name="Centroid Graph (Types)")


