# photopi/point_cloud_analysis/point_cloud/leaf_angles.py

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import logging

from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from sklearn.linear_model import LinearRegression
from sklearn.decomposition import PCA
from scipy.cluster.hierarchy import linkage, fcluster
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

import networkx as nx

# Configure logging
logger = logging.getLogger(__name__)


class LeafAngleAnalyzer:
    def __init__(self, point_cloud):
        self.point_cloud = point_cloud
        self.points = np.asarray(point_cloud.points)
        self.sections = []
        self.sections_analysis = []
        self.continuous_structures = []
        self.graph_structure = {}
        self.centroids_pcd = o3d.geometry.PointCloud()
        self.graph = nx.Graph()
        self.branch_off_nodes = []
        self.all_main_stem_points_up = []
        self.all_main_stem_points_down = []
        self.all_leaf_points = []
        self.angles = []

    def process_point_cloud(self, eps=0.006, min_samples=50, n_sections=80):
        """
        Analyzes the point cloud using DBSCAN and finds continuous structures.
        """
        # Split points into sections
        self.sections = self._split_points_into_sections(self.points, n_sections=n_sections)
        # Analyze sections using DBSCAN
        self.sections_analysis = self._analyze_dbscan_results_3d(self.sections, eps=eps, min_samples=min_samples)
        # Construct graph with branching
        self.graph_structure = self._construct_graph_with_branching(self.sections_analysis, eps=0.05)
        # Find continuous structures
        self.continuous_structures = self._find_continuous_structures(self.graph_structure)

    def aggregate_centroids(self):
        """
        Aggregates centroids from the continuous structures and prepares them for graph construction.
        """
        # Aggregate centroids
        seedling_centroids = [self._aggregate_structure_centroids(self.sections_analysis, structure) for structure in self.continuous_structures]
        # Combine centroids into a single point cloud
        all_centroids = []
        colors = []
        for centroids in seedling_centroids:
            all_centroids.extend(centroids)
            color = np.random.rand(3)
            colors.extend([color] * len(centroids))
        self.centroids_pcd.points = o3d.utility.Vector3dVector(np.array(all_centroids))
        self.centroids_pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

    def build_graph(self, k=2):
        """
        Builds a graph from the centroids using k-NN for connectivity.
        """
        centroids_pcd_points = np.asarray(self.centroids_pcd.points)
        # Add nodes
        for idx, point in enumerate(centroids_pcd_points):
            self.graph.add_node(idx, pos=point)
        # Use k-NN to find nearest neighbors
        nbrs = NearestNeighbors(n_neighbors=k+1, algorithm='ball_tree').fit(centroids_pcd_points)
        distances, indices = nbrs.kneighbors(centroids_pcd_points)
        # Add edges
        for i in range(len(centroids_pcd_points)):
            for j in range(1, k+1):
                self.graph.add_edge(i, indices[i][j], weight=distances[i][j])

    def segment_nodes(self):
        """
        Segments nodes into 'stem' and 'branch_or_leaf' based on distance from the main axis.
        """
        # Extract positions
        pos = np.array([self.graph.nodes[node]['pos'] for node in self.graph.nodes()])
        # Perform PCA
        pca = PCA(n_components=3)
        pca.fit(pos)
        main_axis = pca.components_[0]
        # Project onto main axis
        projections = np.dot(pos, main_axis)
        # Calculate distances from main axis
        distances_from_axis = np.linalg.norm(pos - np.outer(projections, main_axis), axis=1)
        # Threshold
        stem_threshold = np.percentile(distances_from_axis, 75)
        # Assign node types
        for i, node in enumerate(self.graph.nodes()):
            if distances_from_axis[i] < stem_threshold:
                self.graph.nodes[node]['type'] = 'stem'
            else:
                self.graph.nodes[node]['type'] = 'branch_or_leaf'

    def refine_segmentation(self):
        """
        Refines node segmentation using hierarchical clustering.
        """
        pos = np.array([self.graph.nodes[node]['pos'] for node in self.graph.nodes()])
        # Hierarchical clustering
        Z = linkage(pos, method='ward')
        cluster_labels = fcluster(Z, t=20, criterion='distance')
        # Map clusters to nodes
        for i, node in enumerate(self.graph.nodes()):
            self.graph.nodes[node]['cluster'] = cluster_labels[i]
        # Reclassify based on cluster size
        for cluster_label in np.unique(cluster_labels):
            cluster_nodes = [node for node in self.graph.nodes() if self.graph.nodes[node]['cluster'] == cluster_label]
            if len(cluster_nodes) < 10:
                for node in cluster_nodes:
                    self.graph.nodes[node]['type'] = 'branch_or_leaf'

    def remove_cycles(self):
        """
        Detects and removes cycles in the graph to prevent loops.
        """
        cycles = list(nx.cycle_basis(self.graph))
        for cycle in cycles:
            max_edge = None
            max_weight = -np.inf
            for i in range(len(cycle)):
                edge = (cycle[i], cycle[(i + 1) % len(cycle)])
                weight = self.graph[edge[0]][edge[1]]['weight']
                if weight > max_weight:
                    max_edge = edge
                    max_weight = weight
            if max_edge:
                self.graph.remove_edge(*max_edge)

    def label_branch_off_nodes(self):
        """
        Labels nodes with degree 3 as 'branch_off'.
        """
        for node in self.graph.nodes():
            node_degree = self.graph.degree(node)
            if node_degree == 3:
                self.graph.nodes[node]['type'] = 'branch_off'
        # Collect branch-off nodes
        self.branch_off_nodes = [node for node in self.graph.nodes() if self.graph.nodes[node]['type'] == 'branch_off']

    def refine_classification(self, iterations=50):
        """
        Refines node classification over several iterations.
        """
        for _ in range(iterations):
            new_labels = {}
            for node in self.graph.nodes():
                if self.graph.nodes[node]['type'] == 'stem':
                    neighbors = list(self.graph.neighbors(node))
                    stem_count = sum(1 for neighbor in neighbors if self.graph.nodes[neighbor]['type'] == 'stem')
                    branch_count = len(neighbors) - stem_count
                    if branch_count > stem_count:
                        new_labels[node] = 'branch_or_leaf'
                    else:
                        new_labels[node] = 'stem'
            for node, new_label in new_labels.items():
                self.graph.nodes[node]['type'] = new_label

    def reclassify_stem_nodes(self):
        """
        Reclassification of main stem nodes between branch-off and branch/leaf nodes.
        """
        for node in self.graph.nodes():
            if self.graph.nodes[node]['type'] == 'branch_off':
                # Start a breadth-first search (BFS) from the branch-off node
                visited = set()
                queue = [(node, [])]  # (current_node, path_of_stem_nodes)
                while queue:
                    current_node, stem_path = queue.pop(0)
                    if current_node in visited:
                        continue
                    visited.add(current_node)
                    # If we encounter another branch-off node, stop and do not reclassify
                    if self.graph.nodes[current_node]['type'] == 'branch_off' and current_node != node:
                        continue
                    # If we encounter a branch/leaf node, reclassify all previous main stem nodes in the path
                    if self.graph.nodes[current_node]['type'] == 'branch_or_leaf':
                        for stem_node in stem_path:
                            self.graph.nodes[stem_node]['type'] = 'branch_or_leaf'
                    # If we encounter a main stem node, add it to the stem path
                    if self.graph.nodes[current_node]['type'] == 'stem':
                        stem_path.append(current_node)
                    # Explore all neighbors of the current node
                    for neighbor in self.graph.neighbors(current_node):
                        if neighbor not in visited:
                            queue.append((neighbor, stem_path.copy()))

    def extract_sections(self, n_main_stem=5, n_leaf=5):
        """
        Extracts main stem and leaf points for each branch-off node.
        """
        for branch_off_node in self.branch_off_nodes:
            # Get the neighbors of the branch_off_node (should be two main stem nodes and one leaf node)
            neighbors = list(self.graph.neighbors(branch_off_node))
            main_stem_neighbors = [n for n in neighbors if self.graph.nodes[n]['type'] == 'stem']
            leaf_neighbors = [n for n in neighbors if self.graph.nodes[n]['type'] == 'branch_or_leaf']

            # Ensure there are exactly two main stem neighbors and one leaf neighbor
            if len(main_stem_neighbors) != 2 or len(leaf_neighbors) != 1:
                logger.warning(f"Branch-off node {branch_off_node} does not have exactly two main stem neighbors and one leaf neighbor.")
                continue

            # Initialize storage for the points of this branch-off node
            main_stem_points_up = []
            main_stem_points_down = []
            leaf_points = []

            # Step 1: Traverse the first main stem neighbor (upwards path)
            stem_nodes_up = self._traverse_stem(main_stem_neighbors[0], branch_off_node, n_main_stem)

            # Step 2: Traverse the second main stem neighbor (downwards path)
            stem_nodes_down = self._traverse_stem(main_stem_neighbors[1], branch_off_node, n_main_stem)

            # Step 3: Traverse the leaf neighbor
            leaf_nodes = self._traverse_leaf(leaf_neighbors[0], branch_off_node, n_leaf)

            # Convert nodes to points for the current branch-off node
            for node in stem_nodes_up:
                main_stem_points_up.append(self.graph.nodes[node]['pos'])

            for node in stem_nodes_down:
                main_stem_points_down.append(self.graph.nodes[node]['pos'])

            for node in leaf_nodes:
                leaf_points.append(self.graph.nodes[node]['pos'])

            # Store the points
            self.all_main_stem_points_up.append(np.array(main_stem_points_up))
            self.all_main_stem_points_down.append(np.array(main_stem_points_down))
            self.all_leaf_points.append(np.array(leaf_points))

    def compute_leaf_angles(self):
        """
        Computes the angles between the main stem and leaves at each branch-off point.
        """
        for i in range(len(self.all_main_stem_points_up)):
            # Combine stem points
            main_stem_combined = np.vstack((self.all_main_stem_points_up[i], self.all_main_stem_points_down[i]))
            # Perform linear regression
            x_stem, y_stem, z_stem = perform_linear_regression(main_stem_combined)
            x_leaf, y_leaf, z_leaf = perform_linear_regression(self.all_leaf_points[i])
            # Create direction vectors
            stem_vector = np.array([x_stem[-1], y_stem[-1], z_stem[-1]]) - np.array([x_stem[0], y_stem[0], z_stem[0]])
            leaf_vector = np.array([x_leaf[-1], y_leaf[-1], z_leaf[-1]]) - np.array([x_leaf[0], y_leaf[0], z_leaf[0]])
            # Calculate angle
            angle = calculate_angle_between_vectors(stem_vector, leaf_vector)
            self.angles.append(angle)
            print(f"Branch-off {i+1}: Leaf angle = {angle:.2f} degrees")

    def visualize_results(self):
        """
        Visualizes the leaf angles with extended lines, arcs, and angle annotations.
        """
        geometries = []
        for i in range(len(self.all_main_stem_points_up)):
            main_stem_combined = np.vstack((self.all_main_stem_points_up[i], self.all_main_stem_points_down[i]))
            x_stem, y_stem, z_stem = perform_linear_regression(main_stem_combined)
            x_leaf, y_leaf, z_leaf = perform_linear_regression(self.all_leaf_points[i])
            stem_vector = np.array([x_stem[-1], y_stem[-1], z_stem[-1]]) - np.array([x_stem[0], y_stem[0], z_stem[0]])
            leaf_vector = np.array([x_leaf[-1], y_leaf[-1], z_leaf[-1]]) - np.array([x_leaf[0], y_leaf[0], z_leaf[0]])
            extended_stem_end = extend_vector(np.array([x_stem[0], y_stem[0], z_stem[0]]), stem_vector)
            extended_leaf_end = extend_vector(np.array([x_leaf[0], y_leaf[0], z_leaf[0]]), leaf_vector)
            branch_off_pos = np.array([x_stem[0], y_stem[0], z_stem[0]])
            arc_points = create_arc_points(branch_off_pos, extended_stem_end, extended_leaf_end)
            # Create geometries (points, lines, arcs)
            # Main stem
            pcd_main_stem = o3d.geometry.PointCloud()
            pcd_main_stem.points = o3d.utility.Vector3dVector(main_stem_combined)
            pcd_main_stem.paint_uniform_color([1, 0, 0])
            geometries.append(pcd_main_stem)
            # Leaf
            pcd_leaf = o3d.geometry.PointCloud()
            pcd_leaf.points = o3d.utility.Vector3dVector(self.all_leaf_points[i])
            pcd_leaf.paint_uniform_color([0, 1, 0])
            geometries.append(pcd_leaf)
            # Lines
            points = np.vstack([
                [x_stem[0], y_stem[0], z_stem[0]], extended_stem_end,
                [x_leaf[0], y_leaf[0], z_leaf[0]], extended_leaf_end
            ])
            lines = [[0, 1], [2, 3]]
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector([[0, 0, 1], [0, 0, 1]])
            geometries.append(line_set)
            # Arc
            arc_line_set = o3d.geometry.LineSet()
            arc_line_set.points = o3d.utility.Vector3dVector(arc_points)
            arc_lines = [[j, j + 1] for j in range(len(arc_points) - 1)]
            arc_line_set.lines = o3d.utility.Vector2iVector(arc_lines)
            arc_line_set.colors = o3d.utility.Vector3dVector([[1, 0, 1] for _ in arc_lines])
            geometries.append(arc_line_set)
            # Angle text can be added here if Open3D supports text rendering
        # Visualize
        if geometries:
            o3d.visualization.draw_geometries(geometries)
        else:
            print("No geometries to visualize.")

    # Helper methods
    def _split_points_into_sections(self, points, n_sections):
        """
        Splits the point cloud into sections based on Z-coordinate.
        """
        z_values = points[:, 2]
        min_z, max_z = np.min(z_values), np.max(z_values)
        thresholds = np.linspace(min_z, max_z, n_sections + 1)
        sections = []
        for i in range(n_sections):
            section = points[(z_values > thresholds[i]) & (z_values <= thresholds[i + 1])]
            sections.append(section)
        return sections

    def _analyze_dbscan_results_3d(self, sections, eps=0.1, min_samples=3):
        """
        Analyzes sections using DBSCAN clustering.
        """
        sections_analysis = []
        for i, section in enumerate(sections):
            # Apply DBSCAN to the 3D points in the current section
            db = DBSCAN(eps=eps, min_samples=min_samples).fit(section)
            labels = db.labels_
            unique_labels = set(labels)
            section_data = []
            cluster_count = 0
            for k in unique_labels:
                if k == -1:  # Skip noise points
                    continue
                class_member_mask = (labels == k)
                cluster_points = section[class_member_mask]
                # Calculate centroid for this cluster
                centroid = np.mean(cluster_points, axis=0)
                # Calculate the radius
                radius = np.max(np.sqrt(np.sum((cluster_points - centroid) ** 2, axis=1)))
                # Store the cluster data
                cluster_data = {'centroid': centroid, 'radius': radius, 'points': cluster_points}
                section_data.append(cluster_data)
                cluster_count += 1
            # Store the section analysis result
            section_analysis = {'section_index': i, 'clusters': section_data, 'count': cluster_count}
            sections_analysis.append(section_analysis)
        return sections_analysis

    def _construct_graph_with_branching(self, sections_analysis, eps=0.1):
        """
        Constructs a graph from the clustering results, connecting clusters across sections.
        """
        graph = {}
        n_sections = len(sections_analysis)
        for i in range(n_sections - 1):
            current_section = sections_analysis[i]['clusters']
            next_section = sections_analysis[i + 1]['clusters']
            for j, current_cluster in enumerate(current_section):
                current_centroid = current_cluster['centroid']
                connections = []
                for k, next_cluster in enumerate(next_section):
                    next_centroid = next_cluster['centroid']
                    distance = np.linalg.norm(current_centroid - next_centroid)
                    if distance < eps:
                        connections.append((i + 1, k))
                if connections:
                    graph[(i, j)] = connections
        return graph

    def _find_continuous_structures(self, graph):
        """
        Identifies continuous structures in the graph by finding connected components.
        """
        visited = set()
        continuous_structures = []

        def dfs(node, structure):
            if node in visited:
                return
            visited.add(node)
            structure.append(node)
            for neighbor in graph.get(node, []):
                dfs(neighbor, structure)

        for node in graph:
            if node not in visited:
                structure = []
                dfs(node, structure)
                if structure:
                    continuous_structures.append(structure)
        return continuous_structures

    def _aggregate_structure_centroids(self, sections_analysis, structure):
        """
        Aggregates centroids from a continuous structure.
        """
        centroids = []
        for section_index, cluster_index in structure:
            cluster = sections_analysis[section_index]['clusters'][cluster_index]
            centroids.append(cluster['centroid'])
        return centroids

    def _traverse_stem(self, start_node, branch_off_node, n_main_stem):
        """
        Helper method to traverse the main stem nodes.
        """
        current_node = start_node
        visited = set([branch_off_node])
        stem_nodes = []
        while current_node is not None and len(stem_nodes) < n_main_stem:
            if self.graph.nodes[current_node]['type'] == 'stem':
                stem_nodes.append(current_node)
            visited.add(current_node)
            neighbors = [n for n in self.graph.neighbors(current_node) if n not in visited and self.graph.nodes[n]['type'] == 'stem']
            current_node = neighbors[0] if neighbors else None
        return stem_nodes

    def _traverse_leaf(self, start_node, branch_off_node, n_leaf):
        """
        Helper method to traverse the leaf nodes.
        """
        current_node = start_node
        visited = set([branch_off_node])
        leaf_nodes = []
        while current_node is not None and len(leaf_nodes) < n_leaf:
            if self.graph.nodes[current_node]['type'] == 'branch_or_leaf':
                leaf_nodes.append(current_node)
            visited.add(current_node)
            neighbors = [n for n in self.graph.neighbors(current_node) if n not in visited and self.graph.nodes[n]['type'] == 'branch_or_leaf']
            current_node = neighbors[0] if neighbors else None
        return leaf_nodes


# Utility functions
def perform_linear_regression(points):
    """
    Performs linear regression on 3D points using Z as the independent variable.
    """
    X = points[:, 2].reshape(-1, 1)
    Y = points[:, :2]
    reg_x = LinearRegression().fit(X, Y[:, 0])
    reg_y = LinearRegression().fit(X, Y[:, 1])
    z_vals = np.linspace(X.min(), X.max(), 100).reshape(-1, 1)
    x_vals = reg_x.predict(z_vals)
    y_vals = reg_y.predict(z_vals)
    return x_vals, y_vals, z_vals.flatten()


def calculate_angle_between_vectors(v1, v2):
    """
    Calculates the angle between two vectors in degrees.
    """
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    cosine_angle = dot_product / (magnitude_v1 * magnitude_v2)
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return np.degrees(angle)


def extend_vector(start_point, direction_vector, scale=1.5):
    """
    Extends a vector from a start point for visualization purposes.
    """
    extended_point = start_point + scale * direction_vector
    return extended_point


def create_arc_points(center, start_point, end_point, num_points=20):
    """
    Creates points along an arc between two points for visualization.
    """
    v1 = start_point - center
    v2 = end_point - center
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    angle = np.arccos(np.dot(v1_norm, v2_norm))
    arc_points = []
    for t in np.linspace(0, 1, num_points):
        theta = t * angle
        axis = np.cross(v1_norm, v2_norm)
        if np.linalg.norm(axis) == 0:
            # Vectors are parallel; no need to rotate
            arc_point = center + v1
        else:
            axis = axis / np.linalg.norm(axis)
            rotation_matrix = (
                np.cos(theta) * np.eye(3) +
                np.sin(theta) * np.array([[0, -axis[2], axis[1]],
                                          [axis[2], 0, -axis[0]],
                                          [-axis[1], axis[0], 0]]) +
                (1 - np.cos(theta)) * np.outer(axis, axis)
            )
            arc_point = center + np.dot(rotation_matrix, v1)
        arc_points.append(arc_point)
    return np.array(arc_points)


