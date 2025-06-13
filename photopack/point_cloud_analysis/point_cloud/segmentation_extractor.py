import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import logging
import copy
import os


import networkx as nx
from sklearn.neighbors import kneighbors_graph
from scipy.spatial import distance_matrix
from scipy.sparse.csgraph import minimum_spanning_tree

# For advanced cluster detection methods
from sklearn.cluster import KMeans

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Segmentation:
    """
    Segmentation: A class for semi-automatic segmentation and cluster refinement of 3D point clouds.

    This class implements a complete workflow to segment a full-resolution point cloud into distinct 
    clusters using a combination of downsampling, graph-based segmentation, MST-based refinement, and 
    cluster filtering. The resulting segmentation can be used for further analysis, such as morphological 
    measurements or manual refinement.

    The workflow comprises the following major steps:

      1. Preprocessing:
         - Downsample the original point cloud using a voxel filter.
         - Estimate and normalize point normals.
         - Remove statistical outliers from the downsampled cloud.
      
      2. Graph-Based Segmentation:
         - Apply an anisotropic scaling to the z-axis of the downsampled point cloud.
         - Build a k-Nearest Neighbor graph from the scaled points.
         - Prune graph edges that exceed a specified distance threshold.
         - Compute the connected components of the pruned graph and assign a unique cluster label to each component.
         - Store the resulting labels in `self.labels_downsampled`.

      3. MST-Based Refinement:
         - For each (or selected) cluster, compute a distance matrix and build a Minimum Spanning Tree (MST).
         - Cut the largest edges in the MST (up to a specified number of cuts) to refine clusters into subclusters.
         - Update the downsampled labels accordingly.

      4. Cluster Refinement:
         - Remove clusters that are smaller than a given minimum cluster size by marking them as noise (label -1).

      5. Mapping Labels to the Original Point Cloud:
         - Use a KDTree-based nearest neighbor search to transfer the refined labels from the downsampled 
           point cloud back to the original, full-resolution cloud.
         - Store the mapped labels in `self.labels` and preserve the original points and colors in 
           `self.points_array` and `self.colors_array` respectively.

      6. Export and Manual Refinement (Optional):
         - Export individual clusters as separate .ply files for manual segmentation or refinement.
         - Update the labels with manually refined segmentation from external tools.
         - Save and load the final segmented arrays (points, colors, and labels) for later processing.

    Usage Example:
        >>> import open3d as o3d
        >>> pcd = o3d.io.read_point_cloud("plant.ply")
        >>> seg = Segmentation(pcd, min_cluster_size=100)
        >>> seg.preprocess(voxel_size=0.005, nb_neighbors=20, std_ratio=2.0)
        >>> seg.graph_based_segmentation(k=30, z_scale=1.0, distance_threshold=0.02)
        >>> seg.refine_with_mst_segmentation(num_cuts=1)
        >>> seg.refine_clusters(min_cluster_size=100)
        >>> seg.map_labels_to_original()
        >>> seg.segment_and_store()
        >>> seg.visualize_downsampled()
        >>> seg.visualize_segments_original()

    Attributes:
        original_pcd (o3d.geometry.PointCloud): The original, full-resolution point cloud.
        pcd (o3d.geometry.PointCloud): A deep copy of the original point cloud used for processing.
        pcd_downsampled (o3d.geometry.PointCloud): The downsampled version of the point cloud.
        labels_downsampled (np.ndarray): Cluster labels obtained from graph-based segmentation on the downsampled cloud.
        labels (np.ndarray): Cluster labels mapped back to the original point cloud.
        points_array (np.ndarray): The original point cloud's points (stored after mapping labels).
        colors_array (np.ndarray): The original point cloud's colors.
        labels_array (np.ndarray): The final set of labels for the original point cloud.
        min_cluster_size (int): The minimum number of points a cluster must have to be retained.
        
        # For graph-based segmentation:
        sections (list): (Optional) Horizontal slices of the point cloud (if used in another context).
        sections_analysis (list): Results from per-slice DBSCAN clustering (if applicable).
        graph_structure (dict): Adjacency information linking clusters across slices (if applicable).
        continuous_structures (list): Continuous vertical structures identified across slices (if applicable).

        centroids_pcd (o3d.geometry.PointCloud): Aggregated centroids from the clusters, used to build the final graph.
        graph (networkx.Graph): The k-NN graph constructed from the aggregated centroids (after pruning and refinement).
        branch_off_nodes (list): Nodes in the graph identified as branch-off (potential branch connection points).

    """

    def __init__(self, point_cloud, min_cluster_size=100):
        """
        Initialize the Segmentation class.

        Args:
            point_cloud (o3d.geometry.PointCloud): Original, full-resolution point cloud.
            min_cluster_size (int): Default minimum cluster size for refinement steps.
        """
        self.original_pcd = point_cloud
        self.pcd = copy.deepcopy(point_cloud)          # Full-resolution copy
        self.pcd_downsampled = None                    # Downsampled point cloud
        self.labels_downsampled = None                 # Labels for downsampled points
        self.labels = None                             # Labels for original points
        self.min_cluster_size = min_cluster_size

        # Arrays for final storage (original resolution)
        self.points_array = None
        self.colors_array = None
        self.labels_array = None

    # ----------------------------------------------------------------
    # 1. PREPROCESS
    # ----------------------------------------------------------------

    def preprocess(self, voxel_size=0.005, nb_neighbors=20, std_ratio=2.0):
        """
        Downsample and remove outliers from the point cloud.

        Args:
            voxel_size (float): Voxel size for downsampling.
            nb_neighbors (int): Number of neighbors for noise removal.
            std_ratio (float): Standard deviation ratio for noise removal.
        """
        logger.info("Preprocessing: Downsampling & Noise Removal.")

        logger.info(f"Original cloud has {len(self.pcd.points)} points.")
        self.pcd_downsampled = self.pcd.voxel_down_sample(voxel_size=voxel_size)
        logger.info(f"Downsampled to {len(self.pcd_downsampled.points)} points.")

        # Estimate normals (optional)
        self.pcd_downsampled.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        self.pcd_downsampled.normalize_normals()

        # Remove outliers
        cl, _ = self.pcd_downsampled.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        self.pcd_downsampled = cl
        logger.info(f"After noise removal: {len(self.pcd_downsampled.points)} points.")

    # ----------------------------------------------------------------
    # 2. SEGMENTATION
    # ----------------------------------------------------------------

    def graph_based_segmentation(self, k=30, z_scale=1.0, distance_threshold=0.02):
        """
        Graph-based segmentation on the downsampled cloud:
         1) Scale z-axis (anisotropic)
         2) Build k-NN graph
         3) Prune edges > distance_threshold
         4) Connected components => cluster labels

        Args:
            k (int): Number of nearest neighbors.
            z_scale (float): Factor for scaling z-axis (anisotropy).
            distance_threshold (float): Prune edges longer than this.
        """
        if self.pcd_downsampled is None:
            raise RuntimeError("No downsampled point cloud available. Run preprocess() first.")

        logger.info("Starting Graph-Based Segmentation.")
        points = np.asarray(self.pcd_downsampled.points).copy()
        points[:, 2] *= z_scale  # Anisotropic scaling

        A = kneighbors_graph(points, n_neighbors=k, mode='distance', include_self=False, n_jobs=-1)
        G = nx.from_scipy_sparse_matrix(A)

        # Remove edges longer than threshold
        edges_to_remove = [(u, v) for (u, v, d) in G.edges(data=True) if d['weight'] > distance_threshold]
        G.remove_edges_from(edges_to_remove)

        labels_downsampled = np.full(len(points), -1, dtype=int)
        for idx, comp in enumerate(nx.connected_components(G)):
            labels_downsampled[list(comp)] = idx

        self.labels_downsampled = labels_downsampled
        num_clusters = len(set(labels_downsampled)) - (1 if -1 in labels_downsampled else 0)
        logger.info(f"Graph-Based Segmentation produced {num_clusters} clusters.")

    # ----------------------------------------------------------------
    # 3. MST REFINEMENT
    # ----------------------------------------------------------------

    def refine_with_mst_segmentation(self, num_cuts=1, clusters_to_refine=None):
        """
        For each cluster_to_refine, build MST and cut largest edges => subclusters.

        Args:
            num_cuts (int): Number of largest edges to cut per cluster.
            clusters_to_refine (list or None): Which clusters to refine. If None, refine all.
        """
        if self.labels_downsampled is None:
            raise RuntimeError("No downsampled labels found. Run graph_based_segmentation first.")

        unique_labels = set(self.labels_downsampled)
        unique_labels.discard(-1)

        if clusters_to_refine is None:
            clusters_to_refine = unique_labels
        else:
            clusters_to_refine = set(clusters_to_refine) & unique_labels

        max_label = self.labels_downsampled.max()

        for label in clusters_to_refine:
            indices = np.where(self.labels_downsampled == label)[0]
            if len(indices) < 2:
                continue
            sub_pcd = self.pcd_downsampled.select_by_index(indices)
            sub_points = np.asarray(sub_pcd.points)

            dist_mat = distance_matrix(sub_points, sub_points)
            mst = minimum_spanning_tree(dist_mat)
            mst_dense = mst.toarray()

            # Largest edges
            arr_copy = mst_dense.copy()
            arr_copy[arr_copy == 0] = np.nan
            flat = arr_copy.flatten()
            idx_sort_desc = np.argsort(flat)[::-1]
            idx_sort_desc = idx_sort_desc[~np.isnan(flat[idx_sort_desc])]

            actual_cuts = min(num_cuts, len(idx_sort_desc))
            if actual_cuts == 0:
                continue

            cut_indices = np.unravel_index(idx_sort_desc[:actual_cuts], mst_dense.shape)
            mst_dense[cut_indices] = 0
            mst_dense[(cut_indices[1], cut_indices[0])] = 0  # symmetrical

            G_refined = nx.from_numpy_array(mst_dense)
            comps = list(nx.connected_components(G_refined))

            # Assign new labels
            new_labels = np.full(len(sub_points), -1, dtype=int)
            for comp_idx, comp in enumerate(comps):
                max_label += 1
                new_labels[list(comp)] = max_label

            self.labels_downsampled[indices] = new_labels

        final_clusters = set(self.labels_downsampled)
        final_clusters.discard(-1)
        logger.info(f"After MST refinement, we have {len(final_clusters)} clusters.")

    # ----------------------------------------------------------------
    # 4. REFINEMENT (REMOVING OR RE-LABELING SMALL CLUSTERS)
    # ----------------------------------------------------------------

    def refine_clusters(self, min_cluster_size=None):
        """
        Remove clusters smaller than min_cluster_size => label them -1 (noise).
        """
        if self.labels_downsampled is None:
            logger.error("No segmentation labels found on downsampled data. Perform clustering first.")
            return

        if min_cluster_size is None:
            min_cluster_size = self.min_cluster_size

        lbls, counts = np.unique(self.labels_downsampled, return_counts=True)
        size_map = dict(zip(lbls, counts))

        small_clusters = [lbl for lbl, sz in size_map.items() if lbl != -1 and sz < min_cluster_size]
        for c in small_clusters:
            self.labels_downsampled[self.labels_downsampled == c] = -1

        logger.info(f"Removed {len(small_clusters)} clusters below size={min_cluster_size}.")

    # ----------------------------------------------------------------
    # 5. DETECTING “LARGE” CLUSTERS (various methods)
    # ----------------------------------------------------------------

    def get_cluster_sizes_downsampled(self):
        """
        Returns a dict {label: cluster_size} for the downsampled point cloud.
        """
        if self.labels_downsampled is None:
            return {}
        lbls, counts = np.unique(self.labels_downsampled, return_counts=True)
        return dict(zip(lbls, counts))

    def detect_large_clusters(self,
                              method="top_n",
                              top_n=3,
                              percentile=90,
                              stdev_factor=2.0,
                              iqr_factor=1.5,
                              kmeans_clusters=2):
        """
        Return a list of cluster labels considered "large," using one of several methods:
            1) top_n: pick the N largest clusters
            2) percentile: pick clusters above a certain percentile of size
            3) stddev: pick clusters > mean + stdev_factor*std
            4) iqr: pick clusters > Q3 + iqr_factor*(Q3-Q1)
            5) kmeans: group cluster sizes into kmeans_clusters groups; pick group with largest center

        Args:
            method (str): One of ["top_n", "percentile", "stddev", "iqr", "kmeans"].
            top_n (int): For method="top_n".
            percentile (float): For method="percentile", e.g. 90 => 90th percentile.
            stdev_factor (float): For method="stddev".
            iqr_factor (float): For method="iqr".
            kmeans_clusters (int): For method="kmeans".

        Returns:
            A list of cluster labels considered "large."
        """
        size_map = self.get_cluster_sizes_downsampled()
        # Remove noise
        if -1 in size_map:
            size_map.pop(-1)

        if not size_map:
            logger.warning("No non-noise clusters found. Returning empty list.")
            return []

        labels_array = np.array(list(size_map.keys()))
        sizes_array = np.array(list(size_map.values()), dtype=float)

        if method == "top_n":
            # Sort descending, pick top_n
            sorted_pairs = sorted(zip(labels_array, sizes_array), key=lambda x: x[1], reverse=True)
            selected = [lbl for lbl, _sz in sorted_pairs[:top_n]]

        elif method == "percentile":
            cutoff = np.percentile(sizes_array, percentile)
            selected = [lbl for lbl, sz in size_map.items() if sz > cutoff]

        elif method == "stddev":
            mean_sz = np.mean(sizes_array)
            std_sz = np.std(sizes_array)
            threshold = mean_sz + stdev_factor * std_sz
            selected = [lbl for lbl, sz in size_map.items() if sz > threshold]

        elif method == "iqr":
            Q1 = np.percentile(sizes_array, 25)
            Q3 = np.percentile(sizes_array, 75)
            IQR = Q3 - Q1
            cutoff = Q3 + iqr_factor * IQR
            selected = [lbl for lbl, sz in size_map.items() if sz > cutoff]

        elif method == "kmeans":
            # Cluster the cluster sizes themselves
            arr_2d = sizes_array.reshape(-1, 1)
            km = KMeans(n_clusters=kmeans_clusters, random_state=42)
            km.fit(arr_2d)
            # The cluster with the largest center is considered "large"
            centers = km.cluster_centers_.flatten()
            largest_center_idx = np.argmax(centers)
            cluster_assignments = km.labels_
            selected = labels_array[cluster_assignments == largest_center_idx].tolist()

        else:
            logger.error(f"Unknown method '{method}'. Must be one of top_n, percentile, stddev, iqr, kmeans.")
            return []

        logger.info(f"Method '{method}' selected these 'large' clusters: {selected}")
        return selected

    # ----------------------------------------------------------------
    # 6. MAP LABELS BACK TO ORIGINAL
    # ----------------------------------------------------------------

    def map_labels_to_original(self):
        """
        KDTree-based nearest neighbor label assignment from downsampled -> original.
        """
        if self.pcd_downsampled is None or self.labels_downsampled is None:
            raise ValueError("Downsampled data or labels not available for mapping.")

        tree = o3d.geometry.KDTreeFlann(self.pcd_downsampled)
        orig_points = np.asarray(self.pcd.points)
        orig_labels = np.full(len(orig_points), -1, dtype=int)

        for i, pt in enumerate(orig_points):
            _, idx_nn, _ = tree.search_knn_vector_3d(pt, 1)
            orig_labels[i] = self.labels_downsampled[idx_nn[0]]

        self.labels = orig_labels
        logger.info("Labels mapped to the original point cloud.")

    # ----------------------------------------------------------------
    # 7. SEMI-AUTOMATIC WORKFLOW: EXPORT & UPDATE FROM MANUAL
    # ----------------------------------------------------------------

    def export_clusters_for_manual_labeling(self, clusters_to_refine, filename, output_dir):
        """
        Export given cluster labels (from *original* labels) as separate .plys
        so they can be manually split/refine them in Meshlab.

        Args:
            clusters_to_refine (list): List of cluster labels (in self.labels) to export.
            filename (str): Base name for output (e.g. original .ply filename).
            output_dir (str): Directory to save .ply files.
        """
        if self.labels is None:
            logger.error("No labels on the original cloud. Please run map_labels_to_original() first.")
            return

        os.makedirs(output_dir, exist_ok=True)
        base = os.path.splitext(os.path.basename(filename))[0]

        for lbl in clusters_to_refine:
            idx = np.where(self.labels == lbl)[0]
            if len(idx) == 0:
                continue
            pcd_sub = self.pcd.select_by_index(idx)
            out_path = os.path.join(output_dir, f"{base}_cluster_{lbl}.ply")
            o3d.io.write_point_cloud(out_path, pcd_sub)
            logger.info(f"Exported cluster {lbl} to {out_path} with {len(pcd_sub.points)} points.")

    def update_labels_from_manual_segmentation(self, manual_files, base_filename, outdir, min_points=1000):
        """
        Incorporate newly labeled clusters from manual .ply segments, 
        discarding small clusters below min_points.

        Args:
            manual_files (list[str]): Paths to manually segmented .ply files
            base_filename (str): Base name for saving updated .npz
            outdir (str): Directory to save updated .npz
            min_points (int): Minimum number of points to keep a cluster
        """
        if self.points_array is None or self.colors_array is None or self.labels_array is None:
            raise ValueError("Original segmentation not stored. Run segment_and_store() first.")

        # Get the current max label
        max_label = self.labels_array.max()

        # Build a KDTree on the original pcd
        pcd_orig = o3d.geometry.PointCloud()
        pcd_orig.points = o3d.utility.Vector3dVector(self.points_array)
        pcd_orig.colors = o3d.utility.Vector3dVector(self.colors_array)

        tree = o3d.geometry.KDTreeFlann(pcd_orig)
        manual_segmented_indices = set()

        for mfile in manual_files:
            pcd_man = o3d.io.read_point_cloud(mfile)
            pts_man = np.asarray(pcd_man.points)

            if len(pts_man) < min_points:
                logger.info(f"Ignoring {mfile}, cluster too small ({len(pts_man)} < {min_points}).")
                continue

            max_label += 1
            for pt in pts_man:
                _, idx_nn, _ = tree.search_knn_vector_3d(pt, 1)
                self.labels_array[idx_nn[0]] = max_label
                manual_segmented_indices.add(idx_nn[0])

        # Discard any cluster that is smaller than min_points
        unique_lbls, counts = np.unique(self.labels_array, return_counts=True)
        size_map = dict(zip(unique_lbls, counts))
        for lbl, sz in size_map.items():
            if lbl == -1:
                continue
            if sz < min_points:
                self.labels_array[self.labels_array == lbl] = -1

        # Save updated .npz
        os.makedirs(outdir, exist_ok=True)
        out_path = os.path.join(outdir, f"{base_filename}_updated.npz")
        np.savez(out_path,
                 points=self.points_array,
                 colors=self.colors_array,
                 labels=self.labels_array)
        logger.info(f"Updated labels with manual segmentation => {out_path}")

    # ----------------------------------------------------------------
    # 8. STORING / LOADING / VISUALIZATION
    # ----------------------------------------------------------------

    def segment_and_store(self):
        """
        Capture final arrays (points, colors, labels) for the original cloud.
        """
        if self.labels is None:
            raise ValueError("Labels not mapped to original. Call map_labels_to_original() first.")

        self.points_array = np.asarray(self.pcd.points)
        self.colors_array = np.asarray(self.pcd.colors)
        self.labels_array = self.labels

    def save_numpy_arrays(self, output_path):
        """
        Save the final labeled data (points, colors, labels) to a .npz file.
        """
        if any(v is None for v in [self.points_array, self.colors_array, self.labels_array]):
            raise ValueError("Numpy arrays not found. Run segment_and_store() first.")

        np.savez(output_path,
                 points=self.points_array,
                 colors=self.colors_array,
                 labels=self.labels_array)
        logger.info(f"Saved arrays to {output_path}")

    def load_numpy_arrays(self, input_path):
        """
        Load .npz data back into this object (points, colors, labels).
        """
        data = np.load(input_path)
        self.points_array = data['points']
        self.colors_array = data['colors']
        self.labels_array = data['labels']
        logger.info(f"Loaded arrays from {input_path}")

    def visualize_downsampled(self):
        """
        Visualize the downsampled cloud with its labels.
        """
        if self.labels_downsampled is None:
            logger.error("No downsampled labels. Run segmentation first.")
            return
        pcd_copy = copy.deepcopy(self.pcd_downsampled)
        lbls = self.labels_downsampled
        unique_lbls = set(lbls)
        if -1 in unique_lbls:
            unique_lbls.remove(-1)
        cmap = plt.get_cmap("tab20")
        color_lookup = {}
        unique_sorted = sorted(unique_lbls)
        for i, u in enumerate(unique_sorted):
            color_lookup[u] = cmap(i / max(1, len(unique_sorted)))[:3]
        color_list = []
        for l in lbls:
            if l == -1:
                color_list.append([0, 0, 0])
            else:
                color_list.append(color_lookup[l])
        pcd_copy.colors = o3d.utility.Vector3dVector(np.array(color_list))
        o3d.visualization.draw_geometries([pcd_copy], window_name="Downsampled Segments")

    def visualize_segments_original(self):
        """
        Visualize the original cloud with final labels (self.labels).
        """
        if self.labels is None:
            logger.error("No labels mapped to the original cloud.")
            return
        pcd_copy = copy.deepcopy(self.pcd)
        lbls = self.labels
        unique_lbls = set(lbls)
        if -1 in unique_lbls:
            unique_lbls.remove(-1)

        cmap = plt.get_cmap("tab20")
        color_lookup = {}
        unique_sorted = sorted(unique_lbls)
        for i, u in enumerate(unique_sorted):
            color_lookup[u] = cmap(i / max(1, len(unique_sorted)))[:3]
        print(color_lookup)
        color_list = []
        for l in lbls:
            if l == -1:
                color_list.append([0, 0, 0])
            else:
                color_list.append(color_lookup[l])
        pcd_copy.colors = o3d.utility.Vector3dVector(np.array(color_list))
        o3d.visualization.draw_geometries([pcd_copy], window_name="Original Segments")

    def save_segmented_clouds(self, output_dir, base_filename):
        """
        Save each labeled cluster (except noise) as an individual .ply file (original resolution).
        """
        if any(v is None for v in [self.points_array, self.colors_array, self.labels_array]):
            raise ValueError("Need arrays with labels. Run segment_and_store() first.")

        os.makedirs(output_dir, exist_ok=True)
        valid_mask = (self.labels_array != -1)
        lbls = self.labels_array[valid_mask]
        pts = self.points_array[valid_mask]
        cols = self.colors_array[valid_mask]

        for label in np.unique(lbls):
            cluster_idx = np.where(lbls == label)[0]
            pcd_cluster = o3d.geometry.PointCloud()
            pcd_cluster.points = o3d.utility.Vector3dVector(pts[cluster_idx])
            pcd_cluster.colors = o3d.utility.Vector3dVector(cols[cluster_idx])

            file_out = os.path.join(output_dir, f"{base_filename}_cluster_{label}.ply")
            o3d.io.write_point_cloud(file_out, pcd_cluster)
            logger.info(f"Saved cluster {label} => {file_out}")

    def save_combined_segmented_clouds(self, output_dir, base_filename):
        """
        Save a single PLY with all (points + colors). 
        """
        if any(v is None for v in [self.points_array, self.colors_array, self.labels_array]):
            raise ValueError("Need arrays with labels. Run segment_and_store() first.")

        combined_pcd = o3d.geometry.PointCloud()
        combined_pcd.points = o3d.utility.Vector3dVector(self.points_array)
        combined_pcd.colors = o3d.utility.Vector3dVector(self.colors_array)

        os.makedirs(output_dir, exist_ok=True)
        out_file = os.path.join(output_dir, f"{base_filename}_combined.ply")
        o3d.io.write_point_cloud(out_file, combined_pcd)
        logger.info(f"Saved combined segmented cloud => {out_file}")

