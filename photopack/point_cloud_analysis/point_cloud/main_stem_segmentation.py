import copy
import math
import os
import numpy as np
import open3d as o3d
import networkx as nx
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import linear_sum_assignment
from collections import deque
import matplotlib.pyplot as plt
from collections import defaultdict

class MainStemSegmentation:
    """
    MainStemSegmentation: A class to segment the main stem from a full plant 
    point cloud using a graph-based approach. 

    Overview of the pipeline steps:
      1) **Slicing & Clustering**:
         - Slice the point cloud into horizontal sections.
         - Cluster points in each slice using DBSCAN.
         - Merge small or nearby clusters.

      2) **Adjacency Construction**:
         - Build adjacency between consecutive slices using two alternative methods:
            (A) simple distance-based bipartite matching, 
            (B) a vertical matching approach that penalizes lateral offset and angle.

      3) **Bridging Subgraphs & Negative-Cost Graph**:
         - Attempt to unify disconnected subgraphs (bridge them) by incremental distance thresholds.
         - Build a negative-cost DAG (raindrop model) to extract the trunk path.

      4) **Labeling & Refinement**:
         - Mark trunk-path nodes as 'stem', all others as 'leaf'.
         - Identify branch-off nodes by degree threshold, then refine or remove outliers.

      5) **Angle Extraction**:
         - For each branch-off node, gather stem and leaf node coordinates, 
           fit lines, and compute the branch angle relative to the main stem.

    Attributes
    ----------
    original_pcd : o3d.geometry.PointCloud
        The original input point cloud.
    pcd : o3d.geometry.PointCloud
        A working copy of the point cloud (optionally centered or preprocessed).
    points : np.ndarray
        A (N, 3) array of the point cloud data from `pcd`.
    slices : list of np.ndarray
        A list of Z-sliced point subsets, each is Nx3.
    slice_results : list of list of dict
        Results of clustering in each slice. For slice i, 
        slice_results[i] is a list of cluster dicts with keys:
        {'centroid': (3,) float array, 'points': (M,3) float array}.
    n_slices : int
        Number of slices determined or used.
    adjacency_A : dict
        A adjacency dictionary for “method A” (simple bipartite).
    adjacency_B : dict
        A adjacency dictionary for “method B” (vertical-based).
    chosen_adjacency : dict
        The final adjacency chosen (A or B) based on trunk cost.
    cpoints_final : np.ndarray
        (M,3) array of cluster centroids used in the final adjacency method.
    node_map_final : dict
        Mapping from (slice_idx, cluster_idx) -> node_id for final adjacency.
    G_neg_final : nx.DiGraph
        The final negative-cost DAG used for trunk extraction (inverted).
    trunk_path : list of int
        The node IDs forming the trunk path in `G_neg_final`.
    top_node : int
        The top-most node ID (highest z).
    base_node : int
        The base node ID in the trunk path (lowest on the DAG).
    aggregated_centroids_map : list of tuple
        Indexed by node_id => (slice_i, cluster_j).
    branch_off_nodes : list of int
        Node IDs identified as 'branch_off'.
    branch_data : list of dict
        Data of extracted branches => 
        { 'branch_off': <node_id>, 'stem_points': Nx3, 'leaf_points': Mx3, 'angle_degrees': float }.
    labeled_pcd : o3d.geometry.PointCloud
        A final color-labeled point cloud with 'stem' (red) vs 'leaf' (green) in original point space.

    Example
    -------
    >>> pcd = o3d.io.read_point_cloud("myplant.ply")
    >>> segmentation = MainStemSegmentation(pcd)
    >>> segmentation.slice_cloud_z(n_sections=80)
    >>> segmentation.cluster_slices(base_scale=5.0, min_samples=30, dist_merge=0.02, min_pts_in_cluster=15)
    >>> branch_data = segmentation.run_full_pipeline(alpha=1.0, beta=0.5,
    ...                                              raindrop_alpha=1.0, raindrop_beta=1.0,
    ...                                              use_trunk_axis=True, debug=True)
    >>> segmentation.visualize_final_graph_types()
    >>> segmentation.visualize_labeled_pcd()
    """

    def __init__(self, point_cloud: o3d.geometry.PointCloud):
        """
        Initialize with a full plant point cloud.

        Parameters
        ----------
        point_cloud : o3d.geometry.PointCloud
            The input 3D plant point cloud (Open3D object).
        """
        self.original_pcd = point_cloud
        self.pcd = copy.deepcopy(point_cloud)
        self.points = np.asarray(self.pcd.points)

        # Initialize data structures
        self.slices = []
        self.slice_results = []
        self.n_slices = 0

        self.adjacency_A = {}
        self.adjacency_B = {}
        self.chosen_adjacency = {}
        self.cpoints_final = None
        self.node_map_final = {}
        self.G_neg_final = None
        self.trunk_path = []
        self.top_node = None
        self.base_node= None

        self.all_main_stem_points_up   = []
        self.all_main_stem_points_down = []
        self.all_leaf_points           = []

        self.aggregated_centroids_map = []
        self.branch_off_nodes = []
        self.branch_data = []
        self.labeled_pcd = None
        self.output_path = []

    ##########################################################################
    # STEP 1: Slicing & Clustering
    ##########################################################################

    def slice_cloud_z(self, n_sections=None, max_slices=80, show_hist=False):
        """
        Slice the point cloud in Z by dividing [minZ..maxZ] into `n_sections`.

        Parameters
        ----------
        n_sections : int, optional
            If None, we estimate a default from point count (N/500). 
            Else, we clamp to max_slices.
        max_slices : int, default=80
            Maximum number of slices.
        show_hist : bool, default=False
            Whether to display a histogram of the Z distribution (matplotlib).

        Returns
        -------
        slices : list of np.ndarray
            Each element is a subset of points belonging to that Z-slice.
        """
        z_vals = self.points[:, 2]
        z_min, z_max = z_vals.min(), z_vals.max()
        if n_sections is None:
            guess = max(1, len(self.points)//500)
            n_sections = min(guess, max_slices)

        zcuts = np.linspace(z_min, z_max, n_sections + 1)
        slices = []
        for i in range(n_sections):
            mask = (z_vals >= zcuts[i]) & (z_vals < zcuts[i+1])
            slices.append(self.points[mask])

        if show_hist:
            plt.figure()
            plt.hist(z_vals, bins=50)
            plt.title("Z distribution (debug)")
            plt.show()

        self.slices = slices
        self.n_slices = n_sections
        return slices

    def slice_cloud_z_adaptive(self, target_points_per_slice=200, overlap_ratio=0.2, 
                              min_slices=10, max_slices=120, density_threshold=0.5, show_hist=False):
        """
        Advanced adaptive slicing that adjusts slice thickness based on point density and geometry.
        
        This method improves upon the basic slicing by:
        1. Adapting slice thickness to point density variations
        2. Adding overlap between consecutive slices for continuity
        3. Using geometric complexity analysis for better slice positioning
        
        Parameters
        ----------
        target_points_per_slice : int, default=200
            Target number of points per slice for optimal clustering performance.
        overlap_ratio : float, default=0.2
            Fraction of slice thickness to overlap with adjacent slices (0.0-0.5).
        min_slices : int, default=10
            Minimum number of slices regardless of point cloud size.
        max_slices : int, default=120
            Maximum number of slices to prevent over-segmentation.
        density_threshold : float, default=0.5
            Threshold for detecting density variations (relative to mean density).
        show_hist : bool, default=False
            Whether to display histogram and density analysis.
            
        Returns
        -------
        slices : list of np.ndarray
            Each element is a subset of points belonging to that Z-slice.
        slice_info : list of dict
            Metadata for each slice including bounds, density, overlap regions.
        """
        z_vals = self.points[:, 2]
        z_min, z_max = z_vals.min(), z_vals.max()
        z_range = z_max - z_min
        
        if z_range < 1e-6:
            # Degenerate case: all points at same Z
            self.slices = [self.points.copy()]
            self.n_slices = 1
            return self.slices, [{'z_min': z_min, 'z_max': z_max, 'density': len(self.points), 'overlap': False}]
        
        # Step 1: Analyze point density distribution along Z-axis
        n_density_bins = min(100, len(self.points) // 10)
        hist_counts, hist_edges = np.histogram(z_vals, bins=n_density_bins)
        hist_centers = (hist_edges[:-1] + hist_edges[1:]) / 2
        bin_width = hist_edges[1] - hist_edges[0]
        
        # Smooth density for better analysis
        from scipy.ndimage import gaussian_filter1d
        smooth_counts = gaussian_filter1d(hist_counts.astype(float), sigma=1.0)
        
        # Step 2: Estimate optimal number of slices based on density variation
        mean_density = np.mean(smooth_counts)
        density_variation = np.std(smooth_counts) / (mean_density + 1e-6)
        
        # Base slice count from target points per slice
        base_slices = max(min_slices, len(self.points) // target_points_per_slice)
        
        # Adjust for density variation - more slices in variable density regions
        density_factor = 1.0 + density_variation * 2.0
        adaptive_slices = int(base_slices * density_factor)
        adaptive_slices = np.clip(adaptive_slices, min_slices, max_slices)
        
        # Step 3: Create adaptive slice boundaries
        # Use cumulative point distribution for better slice balancing
        sorted_z = np.sort(z_vals)
        cumulative_points = np.arange(1, len(sorted_z) + 1)
        points_per_slice = len(sorted_z) / adaptive_slices
        
        slice_boundaries = []
        slice_info = []
        
        for i in range(adaptive_slices + 1):
            if i == 0:
                z_boundary = z_min
            elif i == adaptive_slices:
                z_boundary = z_max
            else:
                target_point_idx = int(i * points_per_slice)
                target_point_idx = min(target_point_idx, len(sorted_z) - 1)
                z_boundary = sorted_z[target_point_idx]
            slice_boundaries.append(z_boundary)
        
        # Step 4: Create slices with overlap
        slices = []
        overlap_amount = overlap_ratio * (z_range / adaptive_slices)
        
        for i in range(adaptive_slices):
            z_start = slice_boundaries[i]
            z_end = slice_boundaries[i + 1]
            
            # Add overlap
            if overlap_ratio > 0:
                if i > 0:
                    z_start -= overlap_amount / 2
                if i < adaptive_slices - 1:
                    z_end += overlap_amount / 2
            
            # Extract points in this slice
            mask = (z_vals >= z_start) & (z_vals <= z_end)
            slice_points = self.points[mask]
            slices.append(slice_points)
            
            # Calculate slice density and metadata
            slice_density = len(slice_points) / ((z_end - z_start) + 1e-6)
            slice_info.append({
                'z_min': z_start,
                'z_max': z_end,
                'z_center': (z_start + z_end) / 2,
                'n_points': len(slice_points),
                'density': slice_density,
                'has_overlap': overlap_ratio > 0,
                'slice_index': i
            })
        
        if show_hist:
            plt.figure(figsize=(12, 8))
            
            # Plot density distribution
            plt.subplot(2, 2, 1)
            plt.hist(z_vals, bins=50, alpha=0.7, color='blue', label='Point distribution')
            plt.axhline(y=mean_density, color='red', linestyle='--', label=f'Mean density: {mean_density:.1f}')
            plt.xlabel('Z coordinate')
            plt.ylabel('Point count')
            plt.title('Z distribution analysis')
            plt.legend()
            
            # Plot slice boundaries
            plt.subplot(2, 2, 2)
            for i, boundary in enumerate(slice_boundaries):
                plt.axvline(x=boundary, color='red', alpha=0.6)
                if i < len(slice_info):
                    plt.text(boundary, i, f'S{i}', rotation=90, fontsize=8)
            plt.hist(z_vals, bins=50, alpha=0.5, color='green')
            plt.xlabel('Z coordinate')
            plt.title(f'Adaptive slicing ({adaptive_slices} slices)')
            
            # Plot slice densities
            plt.subplot(2, 2, 3)
            slice_centers = [info['z_center'] for info in slice_info]
            slice_densities = [info['density'] for info in slice_info]
            plt.plot(slice_centers, slice_densities, 'o-', color='purple', label='Slice density')
            plt.axhline(y=np.mean(slice_densities), color='orange', linestyle='--', label='Mean slice density')
            plt.xlabel('Z coordinate')
            plt.ylabel('Points per unit height')
            plt.title('Slice density distribution')
            plt.legend()
            
            # Plot slice point counts
            plt.subplot(2, 2, 4)
            slice_indices = [info['slice_index'] for info in slice_info]
            slice_counts = [info['n_points'] for info in slice_info]
            plt.bar(slice_indices, slice_counts, alpha=0.7, color='cyan')
            plt.axhline(y=target_points_per_slice, color='red', linestyle='--', 
                       label=f'Target: {target_points_per_slice}')
            plt.xlabel('Slice index')
            plt.ylabel('Point count')
            plt.title('Points per slice')
            plt.legend()
            
            plt.tight_layout()
            plt.show()
            
            print(f"Adaptive slicing results:")
            print(f"  - Created {adaptive_slices} slices (target: {target_points_per_slice} pts/slice)")
            print(f"  - Density variation factor: {density_factor:.2f}")
            print(f"  - Overlap ratio: {overlap_ratio:.1%}")
            print(f"  - Average points per slice: {np.mean(slice_counts):.1f} ± {np.std(slice_counts):.1f}")
        
        self.slices = slices
        self.n_slices = adaptive_slices
        return slices, slice_info

    def cluster_slices(self, base_scale=5.0, min_samples=30, dist_merge=0.02, min_pts_in_cluster=15):
        """
        For each Z-slice, run DBSCAN, then merge close clusters and unify small clusters.

        Parameters
        ----------
        base_scale : float
            Scale factor to multiply the median neighbor distance for DBSCAN's eps.
        min_samples : int
            Minimum samples in DBSCAN.
        dist_merge : float
            Threshold to merge cluster centroids within the same slice.
        min_pts_in_cluster : int
            Minimum points required to keep a cluster separate; else unify with nearest.

        Returns
        -------
        slice_results : list of list of dict
            For each slice i, a list of cluster dicts { 'centroid':..., 'points':... }.
        """
        if not self.slices:
            print("[WARN] No slices found => call slice_cloud_z first.")
            return []

        slice_results = []
        for s_pts in self.slices:
            if len(s_pts) < min_samples:
                # Not enough points => skip or store empty
                slice_results.append([])
                continue

            # compute eps via base_scale * median_distance
            nbrs = NearestNeighbors(n_neighbors=2).fit(s_pts)
            dists,_ = nbrs.kneighbors(s_pts)
            median_dist = np.median(dists[:,1])
            eps_val = base_scale * median_dist

            db = DBSCAN(eps=eps_val, min_samples=min_samples).fit(s_pts)
            labels = db.labels_

            # Gather clusters
            cluster_list = []
            for lbl in set(labels):
                if lbl == -1:
                    continue
                mask = (labels == lbl)
                cpts = s_pts[mask]
                cent = cpts.mean(axis=0)
                cluster_list.append({'centroid': cent, 'points': cpts})

            # Merge clusters within dist_merge
            merged = self._merge_in_slice_clusters(cluster_list, dist_thresh=dist_merge)
            final  = self._unify_small_clusters(merged, min_pts_in_cluster=min_pts_in_cluster)
            slice_results.append(final)

        self.slice_results = slice_results
        return slice_results
    
    def cluster_slices_adaptive(self, base_scale=5.0, min_samples=20, dist_merge=0.02, 
                               min_pts_in_cluster=15, density_factor=2.0, multi_scale_levels=3,
                               outlier_threshold=0.1, debug=False):
        """
        Advanced adaptive clustering that uses multi-scale DBSCAN with density-based eps adjustment.
        
        This method improves upon basic clustering by:
        1. Analyzing local density variations within each slice
        2. Using multiple eps values for different density regions
        3. Implementing hierarchical clustering for complex regions
        4. Better handling of outliers and noise
        
        Parameters
        ----------
        base_scale : float, default=5.0
            Base scale factor for eps calculation.
        min_samples : int, default=20
            Minimum samples for DBSCAN core points.
        dist_merge : float, default=0.02
            Threshold to merge cluster centroids within the same slice.
        min_pts_in_cluster : int, default=15
            Minimum points required to keep a cluster separate.
        density_factor : float, default=2.0
            Factor for adjusting eps based on local density variations.
        multi_scale_levels : int, default=3
            Number of different scales to try for clustering.
        outlier_threshold : float, default=0.1
            Threshold for detecting outlier points (relative to slice size).
        debug : bool, default=False
            Whether to print debugging information.
            
        Returns
        -------
        slice_results : list of list of dict
            Enhanced clustering results with additional metadata.
        clustering_stats : dict
            Statistics about the clustering process.
        """
        if not self.slices:
            print("[WARN] No slices found => call slice_cloud_z or slice_cloud_z_adaptive first.")
            return [], {}
        
        slice_results = []
        clustering_stats = {
            'total_slices': len(self.slices),
            'processed_slices': 0,
            'total_clusters': 0,
            'outlier_points': 0,
            'multi_scale_used': 0,
            'avg_eps_values': [],
            'slice_complexities': []
        }
        
        for slice_idx, s_pts in enumerate(self.slices):
            if len(s_pts) < min_samples:
                slice_results.append([])
                continue
                
            if debug:
                print(f"[ADAPTIVE-CLUSTER] Processing slice {slice_idx} with {len(s_pts)} points")
                
            # Step 1: Analyze local density variations
            density_analysis = self._analyze_slice_density(s_pts, debug=debug)
            
            # Step 2: Determine optimal clustering strategy
            if density_analysis['complexity'] > 0.5:  # High complexity
                # Use multi-scale clustering
                clusters = self._multi_scale_clustering(
                    s_pts, base_scale, min_samples, multi_scale_levels, 
                    density_analysis, debug=debug
                )
                clustering_stats['multi_scale_used'] += 1
            else:
                # Use adaptive single-scale clustering
                clusters = self._adaptive_single_scale_clustering(
                    s_pts, base_scale, min_samples, density_analysis, debug=debug
                )
            
            # Step 3: Post-process clusters
            if clusters:
                # Merge close clusters
                merged = self._merge_in_slice_clusters(clusters, dist_thresh=dist_merge)
                
                # Unify small clusters with intelligent neighbor selection
                final = self._unify_small_clusters_enhanced(
                    merged, min_pts_in_cluster, density_analysis
                )
                
                # Handle outliers
                final, outliers = self._handle_outliers(final, s_pts, outlier_threshold)
                clustering_stats['outlier_points'] += len(outliers)
                
                slice_results.append(final)
                clustering_stats['total_clusters'] += len(final)
            else:
                slice_results.append([])
            
            clustering_stats['processed_slices'] += 1
            clustering_stats['slice_complexities'].append(density_analysis.get('complexity', 0))
            
        self.slice_results = slice_results
        
        if debug:
            print(f"[ADAPTIVE-CLUSTER] Results summary:")
            print(f"  - Processed {clustering_stats['processed_slices']}/{clustering_stats['total_slices']} slices")
            print(f"  - Created {clustering_stats['total_clusters']} clusters total")
            print(f"  - Multi-scale used in {clustering_stats['multi_scale_used']} slices")
            print(f"  - Handled {clustering_stats['outlier_points']} outlier points")
            print(f"  - Average complexity: {np.mean(clustering_stats['slice_complexities']):.3f}")
        
        return slice_results, clustering_stats
    
    def _analyze_slice_density(self, points, n_regions=8, debug=False):
        """
        Analyze point density variations within a slice to guide clustering strategy.
        
        Returns a dictionary with density analysis results including complexity score.
        """
        if len(points) < 10:
            return {'complexity': 0.0, 'uniform_density': True, 'recommended_eps': None}
        
        # Create spatial grid for density analysis
        xy_points = points[:, :2]  # Only X,Y for slice analysis
        x_min, x_max = xy_points[:, 0].min(), xy_points[:, 0].max()
        y_min, y_max = xy_points[:, 1].min(), xy_points[:, 1].max()
        
        # Avoid degenerate cases
        x_range = max(x_max - x_min, 1e-6)
        y_range = max(y_max - y_min, 1e-6)
        
        # Create grid
        x_edges = np.linspace(x_min, x_max, n_regions + 1)
        y_edges = np.linspace(y_min, y_max, n_regions + 1)
        
        # Calculate density in each grid cell
        densities = []
        valid_cells = 0
        
        for i in range(n_regions):
            for j in range(n_regions):
                mask = ((xy_points[:, 0] >= x_edges[i]) & (xy_points[:, 0] < x_edges[i+1]) &
                       (xy_points[:, 1] >= y_edges[j]) & (xy_points[:, 1] < y_edges[j+1]))
                
                cell_count = np.sum(mask)
                cell_area = (x_edges[i+1] - x_edges[i]) * (y_edges[j+1] - y_edges[j])
                cell_density = cell_count / cell_area
                
                if cell_count > 0:
                    densities.append(cell_density)
                    valid_cells += 1
        
        if len(densities) < 2:
            return {'complexity': 0.0, 'uniform_density': True, 'recommended_eps': None}
        
        # Calculate complexity metrics
        mean_density = np.mean(densities)
        std_density = np.std(densities)
        cv_density = std_density / (mean_density + 1e-6)  # Coefficient of variation
        
        # Estimate optimal eps using k-distance graph approach
        nbrs = NearestNeighbors(n_neighbors=min(5, len(points))).fit(points)
        distances, _ = nbrs.kneighbors(points)
        k_distances = np.sort(distances[:, -1])  # k=5 distances
        
        # Find elbow point for eps estimation
        knee_idx = len(k_distances) // 4  # Simple heuristic
        recommended_eps = k_distances[knee_idx]
        
        complexity = min(1.0, cv_density)  # Normalize to [0,1]
        
        analysis = {
            'complexity': complexity,
            'uniform_density': cv_density < 0.3,
            'mean_density': mean_density,
            'std_density': std_density,
            'cv_density': cv_density,
            'recommended_eps': recommended_eps,
            'n_density_regions': valid_cells,
            'k_distances': k_distances
        }
        
        if debug:
            print(f"    Density analysis: complexity={complexity:.3f}, CV={cv_density:.3f}, recommended_eps={recommended_eps:.4f}")
        
        return analysis
    
    def _adaptive_single_scale_clustering(self, points, base_scale, min_samples, density_analysis, debug=False):
        """Single-scale clustering with density-adapted eps."""
        if density_analysis['recommended_eps'] is not None:
            eps_val = base_scale * density_analysis['recommended_eps']
        else:
            # Fallback to median distance method
            nbrs = NearestNeighbors(n_neighbors=2).fit(points)
            dists, _ = nbrs.kneighbors(points)
            median_dist = np.median(dists[:, 1])
            eps_val = base_scale * median_dist
        
        # Adjust min_samples based on density complexity
        adaptive_min_samples = max(min_samples // 2, 
                                 int(min_samples * (1 - density_analysis['complexity'])))
        
        db = DBSCAN(eps=eps_val, min_samples=adaptive_min_samples).fit(points)
        labels = db.labels_
        
        # Convert to cluster list
        clusters = []
        for lbl in set(labels):
            if lbl == -1:  # Skip noise points
                continue
            mask = (labels == lbl)
            cpts = points[mask]
            cent = cpts.mean(axis=0)
            clusters.append({
                'centroid': cent, 
                'points': cpts,
                'method': 'adaptive_single',
                'eps_used': eps_val
            })
        
        if debug:
            n_noise = np.sum(labels == -1)
            print(f"    Single-scale: eps={eps_val:.4f}, min_samples={adaptive_min_samples}, "
                  f"clusters={len(clusters)}, noise={n_noise}")
        
        return clusters
    
    def _multi_scale_clustering(self, points, base_scale, min_samples, n_scales, density_analysis, debug=False):
        """Multi-scale clustering for complex density regions."""
        base_eps = density_analysis.get('recommended_eps', 0.01)
        if base_eps is None or base_eps < 1e-6:
            nbrs = NearestNeighbors(n_neighbors=2).fit(points)
            dists, _ = nbrs.kneighbors(points)
            base_eps = np.median(dists[:, 1])
        
        # Create multiple scales
        scales = [base_scale * (0.5 ** i) for i in range(n_scales)]
        all_clusters = []
        used_points = set()
        
        for scale_idx, scale in enumerate(scales):
            eps_val = scale * base_eps
            
            # Only cluster unused points
            remaining_points = []
            remaining_indices = []
            for i, pt in enumerate(points):
                if i not in used_points:
                    remaining_points.append(pt)
                    remaining_indices.append(i)
            
            if len(remaining_points) < min_samples:
                break
                
            remaining_points = np.array(remaining_points)
            
            # Adjust min_samples for this scale
            scale_min_samples = max(min_samples // (scale_idx + 1), 3)
            
            db = DBSCAN(eps=eps_val, min_samples=scale_min_samples).fit(remaining_points)
            labels = db.labels_
            
            # Process clusters from this scale
            scale_clusters = 0
            for lbl in set(labels):
                if lbl == -1:
                    continue
                    
                mask = (labels == lbl)
                cluster_pts = remaining_points[mask]
                cluster_indices = [remaining_indices[i] for i in range(len(mask)) if mask[i]]
                
                # Mark these points as used
                used_points.update(cluster_indices)
                
                cent = cluster_pts.mean(axis=0)
                all_clusters.append({
                    'centroid': cent,
                    'points': cluster_pts,
                    'method': f'multi_scale_{scale_idx}',
                    'eps_used': eps_val,
                    'scale_level': scale_idx
                })
                scale_clusters += 1
            
            if debug:
                n_noise = np.sum(labels == -1)
                print(f"    Scale {scale_idx}: eps={eps_val:.4f}, clusters={scale_clusters}, "
                      f"points_processed={len(remaining_points)}, noise={n_noise}")
        
        return all_clusters
    
    def _unify_small_clusters_enhanced(self, clusters, min_pts_in_cluster, density_analysis):
        """Enhanced small cluster unification using density-aware distance metrics."""
        if not clusters:
            return clusters
            
        final = []
        small = []
        
        for cluster in clusters:
            if len(cluster['points']) >= min_pts_in_cluster:
                final.append(cluster)
            else:
                small.append(cluster)
        
        if not final:
            return clusters  # Keep small clusters if no large ones exist
        
        # Unify small clusters with nearest large cluster using enhanced distance metric
        for small_cluster in small:
            best_dist = float('inf')
            best_idx = -1
            small_centroid = small_cluster['centroid']
            
            for idx, large_cluster in enumerate(final):
                large_centroid = large_cluster['centroid']
                
                # Enhanced distance metric considering density
                euclidean_dist = np.linalg.norm(large_centroid - small_centroid)
                
                # Weight by relative cluster sizes (prefer similar-sized clusters)
                size_factor = abs(len(large_cluster['points']) - len(small_cluster['points'])) / max(len(large_cluster['points']), 1)
                size_penalty = 1 + 0.5 * size_factor
                
                # Weight by local density (prefer clusters in similar density regions)
                density_factor = 1.0
                if not density_analysis.get('uniform_density', True):
                    density_factor = 1 + 0.3 * density_analysis.get('complexity', 0)
                
                weighted_dist = euclidean_dist * size_penalty * density_factor
                
                if weighted_dist < best_dist:
                    best_dist = weighted_dist
                    best_idx = idx
            
            if best_idx >= 0:
                # Merge with best cluster
                merged_pts = np.vstack([final[best_idx]['points'], small_cluster['points']])
                new_centroid = merged_pts.mean(axis=0)
                final[best_idx]['points'] = merged_pts
                final[best_idx]['centroid'] = new_centroid
        
        return final
    
    def _handle_outliers(self, clusters, original_points, outlier_threshold):
        """Detect and handle outlier points that weren't clustered properly."""
        if not clusters:
            return clusters, []
        
        # Find points that are in clusters
        clustered_points = set()
        for cluster in clusters:
            for point in cluster['points']:
                clustered_points.add(tuple(point))
        
        # Find outlier points
        outliers = []
        total_points = len(original_points)
        outlier_limit = int(total_points * outlier_threshold)
        
        for point in original_points:
            if tuple(point) not in clustered_points:
                outliers.append(point)
        
        # If we have too many outliers, try to rescue some by relaxed clustering
        if len(outliers) > outlier_limit and len(outliers) > 5:
            rescued_clusters = self._rescue_outliers(outliers)
            clusters.extend(rescued_clusters)
            
            # Update outlier list
            new_outliers = []
            rescued_points = set()
            for cluster in rescued_clusters:
                for point in cluster['points']:
                    rescued_points.add(tuple(point))
            
            for point in outliers:
                if tuple(point) not in rescued_points:
                    new_outliers.append(point)
            outliers = new_outliers
        
        return clusters, outliers
    
    def _rescue_outliers(self, outliers, min_cluster_size=3):
        """Try to form clusters from outlier points using relaxed criteria."""
        if len(outliers) < min_cluster_size:
            return []
        
        outlier_array = np.array(outliers)
        
        # Use very relaxed DBSCAN parameters
        nbrs = NearestNeighbors(n_neighbors=min(3, len(outliers))).fit(outlier_array)
        distances, _ = nbrs.kneighbors(outlier_array)
        eps_val = np.percentile(distances[:, -1], 75)  # Use 75th percentile
        
        db = DBSCAN(eps=eps_val, min_samples=min_cluster_size).fit(outlier_array)
        labels = db.labels_
        
        rescued_clusters = []
        for lbl in set(labels):
            if lbl == -1:
                continue
            mask = (labels == lbl)
            cluster_pts = outlier_array[mask]
            if len(cluster_pts) >= min_cluster_size:
                centroid = cluster_pts.mean(axis=0)
                rescued_clusters.append({
                    'centroid': centroid,
                    'points': cluster_pts,
                    'method': 'rescued_outliers',
                    'eps_used': eps_val
                })
        
        return rescued_clusters

    def _merge_in_slice_clusters(self, slice_clusters, dist_thresh=0.02):
        """
        Merge clusters in a single slice whose centroids are < dist_thresh.

        Internal helper used in `cluster_slices`.
        """
        used = set()
        merged = []
        for i in range(len(slice_clusters)):
            if i in used:
                continue
            cA_pts  = [slice_clusters[i]['points']]
            cA_cent = slice_clusters[i]['centroid']
            for j in range(i+1, len(slice_clusters)):
                if j in used:
                    continue
                cB_cent = slice_clusters[j]['centroid']
                dist_ij = np.linalg.norm(cA_cent - cB_cent)
                if dist_ij < dist_thresh:
                    cA_pts.append(slice_clusters[j]['points'])
                    used.add(j)
            merged_pts = np.vstack(cA_pts)
            new_cent   = merged_pts.mean(axis=0)
            merged.append({'centroid': new_cent, 'points': merged_pts})
            used.add(i)
        return merged

    def _unify_small_clusters(self, slice_clusters, min_pts_in_cluster=15):
        """
        If a cluster is smaller than min_pts_in_cluster, merge it with the nearest bigger one.
        Internal helper for `cluster_slices`.
        """
        final = []
        small = []
        for cdict in slice_clusters:
            if len(cdict['points']) >= min_pts_in_cluster:
                final.append(cdict)
            else:
                small.append(cdict)
        if not final and small:
            return []

        for sc in small:
            best_dist = float('inf')
            best_idx  = -1
            for idx_fc, fc in enumerate(final):
                dist_ = np.linalg.norm(fc['centroid'] - sc['centroid'])
                if dist_ < best_dist:
                    best_dist = dist_
                    best_idx  = idx_fc
            if best_idx >=0:
                merged_pts = np.vstack([final[best_idx]['points'], sc['points']])
                new_cent   = merged_pts.mean(axis=0)
                final[best_idx]['points']   = merged_pts
                final[best_idx]['centroid'] = new_cent
        return final

    ##########################################################################
    # STEP 2: Building Adjacency
    ##########################################################################

    def build_adjacency_bipartite(self, max_dist=0.1):
        """
        Simple bipartite matching for slices i->i+1 using Euclidean distance < max_dist.

        Parameters
        ----------
        max_dist : float
            Distance threshold for linking clusters in consecutive slices.

        Returns
        -------
        adjacency : dict
            Keys: (slice_idx, cluster_idx), 
            Values: list of neighbors => [(slice_idx+1, cluster_idx2), ...].
        """
        adjacency = {}
        for i in range(len(self.slice_results) - 1):
            currC = self.slice_results[i]
            nextC = self.slice_results[i+1]
            m, n = len(currC), len(nextC)
            if m==0 or n==0:
                continue

            cost_mat = np.zeros((m,n), dtype=float)
            for r in range(m):
                A = currC[r]['centroid']
                for c in range(n):
                    B = nextC[c]['centroid']
                    cost_mat[r,c] = np.linalg.norm(A-B)

            row_inds, col_inds = linear_sum_assignment(cost_mat)
            for rr, cc in zip(row_inds, col_inds):
                dist_val = cost_mat[rr, cc]
                if dist_val< max_dist:
                    adjacency[(i, rr)] = adjacency.get((i, rr), []) + [(i+1, cc)]
        return adjacency

    def build_adjacency_bipartite_vertical(self, trunk_axis, max_dist=1.5, alpha=1.0, beta=2.0):
        """
        Another bipartite matching approach that penalizes horizontal offset + beta * angle to trunk.

        cost = horizontal_offset + beta*angle_to_trunk

        Parameters
        ----------
        trunk_axis : np.ndarray
            A (3,) unit vector representing the approximate vertical axis.
        max_dist : float
            Threshold on the cost for linking clusters between slices.
        alpha : float, optional
            Not used directly here but you can incorporate if you want additional scaling.
        beta : float
            Weight factor multiplying the angle to trunk.

        Returns
        -------
        adjacency : dict
            Similar structure to build_adjacency_bipartite.
        """
        adjacency = {}
        # ensure trunk_axis is pointing "up"
        trunk_axis = trunk_axis / (np.linalg.norm(trunk_axis)+1e-12)
        if trunk_axis[2]<0:
            trunk_axis = -trunk_axis

        for i in range(len(self.slice_results)-1):
            currC = self.slice_results[i]
            nextC = self.slice_results[i+1]
            m, n = len(currC), len(nextC)
            if m==0 or n==0:
                continue

            cost_mat = np.zeros((m,n), dtype=float)
            for r in range(m):
                A = currC[r]['centroid']
                for c in range(n):
                    B = nextC[c]['centroid']
                    vec    = B - A
                    horiz  = np.linalg.norm(vec[:2])
                    mag    = np.linalg.norm(vec)
                    if mag<1e-12:
                        angle = 0.0
                    else:
                        dot_val = max(-1.0, min(1.0, vec.dot(trunk_axis)/mag))
                        angle   = math.acos(dot_val)
                    cost_val = horiz + beta*angle
                    cost_mat[r,c] = cost_val

            row_inds, col_inds = linear_sum_assignment(cost_mat)
            for rr, cc in zip(row_inds, col_inds):
                val = cost_mat[rr, cc]
                if val < max_dist:
                    adjacency[(i, rr)] = adjacency.get((i, rr), []) + [(i+1, cc)]
        return adjacency


    ##########################################################################
    # STEP 3: Building the Cost Graph & Extracting Trunk
    ##########################################################################

    def compute_principal_axis(self):
        """
        Perform PCA on all cluster centroids to find the dominant principal axis.

        Returns
        -------
        axis : np.ndarray (3,)
            The principal axis of the centroids, oriented so that axis[2]>=0.
        """
        all_c= []
        for slc in self.slice_results:
            for cd in slc:
                all_c.append(cd['centroid'])
        if len(all_c)<2:
            return np.array([0,0,1], dtype=float)

        arr = np.array(all_c)
        arr_centered = arr - arr.mean(axis=0)
        _,_,vT = np.linalg.svd(arr_centered, full_matrices=False)
        axis = vT[0]
        if axis[2]<0:
            axis = -axis
        return axis


    def build_cost_graph(self, slice_results, adjacency, trunk_axis, alpha=1.0):
        """
        Build a directed graph with edges weighted by cost = dist + alpha*angle_to_trunk.

        Parameters
        ----------
        slice_results : list of list of dict
            The cluster info (each slice => cluster dicts).
        adjacency : dict
            The adjacency from one of the bipartite building methods.
        trunk_axis : np.ndarray
            (3,) approximate vertical axis. 
        alpha : float
            Additional factor to scale the angle cost.

        Returns
        -------
        G : nx.DiGraph
            The cost graph.
        node_map : dict
            Maps (slice_i, cluster_j) -> node_id (int).
        cpoints : np.ndarray
            (N,3) array of cluster centroids.
        """
        node_map = {}
        cpoints  = []
        nd_id=0
        for i,clusts in enumerate(slice_results):
            for j,cd in enumerate(clusts):
                node_map[(i,j)] = nd_id
                cpoints.append(cd['centroid'])
                nd_id+=1
        cpoints = np.array(cpoints)

        G = nx.DiGraph()
        for nd in range(len(cpoints)):
            G.add_node(nd)

        def costfunc(u,v):
            d = np.linalg.norm(v - u)
            if d<1e-9:
                return 0
            vec = (v - u)/d
            dotv= np.clip(vec.dot(trunk_axis), -1,1)
            angle= math.acos(dotv)
            return d + alpha*angle

        for (i,j), neighs in adjacency.items():
            nd1 = node_map[(i,j)]
            A   = cpoints[nd1]
            for (k,l) in neighs:
                nd2 = node_map[(k,l)]
                B   = cpoints[nd2]
                c_uv= costfunc(A,B)
                c_vu= costfunc(B,A)
                G.add_edge(nd1, nd2, weight=c_uv)
                G.add_edge(nd2, nd1, weight=c_vu)

        return G, node_map, cpoints



    def build_raindrop_negcost_digraph(self, cpoints, adjacency, node_map,
                                    alpha=1.0, beta=1.0, gamma=1.0, delta=2.0, trunk_axis=None,
                                    reverse_z=False, debug=True):
        """
        Build a negative-cost DAG for trunk extraction with branch awareness:
        - If not reverse_z: edge (u->v) if z_v < z_u, cost includes branch point preference
        - If reverse_z:     edge (v->u) if z_v >= z_u, symmetrical logic.

        Parameters
        ----------
        cpoints : np.ndarray
            (N,3) centroids.
        adjacency : dict
            The adjacency dict from bipartite matching.
        node_map : dict
            (slice_i, cluster_j) -> node_id
        alpha : float
            Factor for horizontal cost.
        beta : float
            Factor for angle cost.
        gamma : float
            Factor for vertical preference cost.
        delta : float
            Factor for branch point preference (encourages paths through branch points).
        trunk_axis : np.ndarray or None
            The approximate vertical axis; if None, use [0,0,-1].
        reverse_z : bool
            If True, we invert the direction logic: 
            edges go from higher-z to lower-z.
        debug : bool
            If True, prints extra debug info.

        Returns
        -------
        G : nx.DiGraph
            The negative-cost DAG for trunk extraction.
        """
        # Create the directed graph for path finding
        G = nx.DiGraph()
        
        # Create an undirected graph for branch point identification
        undirected_graph = nx.Graph()
        
        N = len(cpoints)
        
        # Add all nodes to both graphs
        for nd in range(N):
            G.add_node(nd, branch_count=0, is_branch_point=False)
            undirected_graph.add_node(nd)
        
        # Set up reference axis
        if trunk_axis is None:
            ref_axis = np.array([0,0,-1], dtype=float)
        else:
            ref_axis = trunk_axis / (np.linalg.norm(trunk_axis)+1e-12)
            if ref_axis[2]>0:
                ref_axis = -ref_axis
                
        def measure_angle(vec, axis):
            mag = np.linalg.norm(vec)
            if mag<1e-12:
                return 0.0
            dotv = max(-1.0, min(1.0, vec.dot(axis)/mag))
            return math.acos(dotv)
        
        # First pass: Add edges to the undirected graph to identify branch points
        for key, neighbors in adjacency.items():
            ndA = node_map[key]
            for nb in neighbors:
                ndB = node_map[nb]
                undirected_graph.add_edge(ndA, ndB)
        
        # Second pass: Identify branch points
        branch_points = {}
        for nd in undirected_graph.nodes():
            degree = undirected_graph.degree(nd)
            if degree >= 3:
                branch_points[nd] = degree - 2  # Number of branches (excluding trunk connections)
                G.nodes[nd]['branch_count'] = branch_points[nd]
                G.nodes[nd]['is_branch_point'] = True
        
        # Third pass: Add directed edges with costs
        total_edges= 0
        for key, neighbors in adjacency.items():
            ndA = node_map[key]
            zA = cpoints[ndA][2]
            for nb in neighbors:
                ndB = node_map[nb]
                zB = cpoints[ndB][2]

                if not reverse_z:
                    # normal => edge if zB < zA
                    if zB < zA:
                        vec = cpoints[ndB] - cpoints[ndA]
                        horiz = np.linalg.norm(vec[:2])
                        ang = measure_angle(vec, ref_axis)
                        
                        # Add vertical preference cost
                        vertical_diff = abs(zB - zA)
                        vertical_ratio = vertical_diff / (vertical_diff + horiz + 1e-10)
                        vertical_cost = (1 - vertical_ratio) * gamma
                        
                        # Branch point preference
                        branch_bonus = delta * (G.nodes[ndA].get('branch_count', 0) + 
                                            G.nodes[ndB].get('branch_count', 0))
                        
                        # Final cost (negative for longest path algorithm)
                        cost_uv = -(alpha*horiz + beta*ang + vertical_cost + branch_bonus)
                        
                        # Add edge with computed weight
                        G.add_edge(ndA, ndB, weight=cost_uv)
                        total_edges += 1
                
                else:
                    # reversed => edge if zB >= zA => (ndB->ndA)
                    if zB >= zA:
                        vec = cpoints[ndA] - cpoints[ndB]
                        horiz = np.linalg.norm(vec[:2])
                        ang = measure_angle(vec, ref_axis)
                        
                        # Add vertical preference cost
                        vertical_diff = abs(zB - zA)
                        vertical_ratio = vertical_diff / (vertical_diff + horiz + 1e-10)
                        vertical_cost = (1 - vertical_ratio) * gamma
                        
                        # Branch point preference
                        branch_bonus = delta * (G.nodes[ndA].get('branch_count', 0) + 
                                            G.nodes[ndB].get('branch_count', 0))
                        
                        # Final cost
                        cost_vu = -(alpha*horiz + beta*ang + vertical_cost + branch_bonus)
                        
                        # Add edge with computed weight
                        G.add_edge(ndB, ndA, weight=cost_vu)
                        total_edges += 1

        if debug:
            print(f"[DEBUG] build_raindrop_negcost_digraph => #edges= {total_edges}")
            print(f"[DEBUG] branch points detected: {len(branch_points)}")
            
        return G

    def longest_path_in_DAG_negcost(self, G, top_node_id, debug=True):
        """
        Given a negative-cost DAG, find the path that yields minimal sum 
        (which is effectively the 'longest' path in positive sense).
        
        We do a topological sort, then DP to pick min-sum path from top_node_id.
        
        Parameters
        ----------
        G : nx.DiGraph
            The negative-cost DAG.
        top_node_id : int
            The node from which we begin the path (highest z).
        debug : bool
            If True, prints debug info.
        
        Returns
        -------
        base_node : int
            The node at the other end of the minimal-cost path.
        path : list of int
            The sequence of nodes in that trunk path.
        """
        topo_order = list(nx.topological_sort(G))
        if top_node_id not in topo_order:
            # fallback
            return (top_node_id, [top_node_id])
        
        maxDist = {}
        pred = {}
        for nd in G.nodes():
            maxDist[nd] = float('inf')  # we want minimal sum => store inf
            pred[nd] = None
        maxDist[top_node_id] = 0.0
        
        for nd in topo_order:
            if maxDist[nd] == float('inf'):
                continue
            cost_nd = maxDist[nd]
            for nb in G[nd]:
                w = G[nd][nb]['weight']
                alt = cost_nd + w
                if alt < maxDist[nb]:
                    maxDist[nb] = alt
                    pred[nb] = nd
        
        best_node = top_node_id
        best_val = maxDist[top_node_id]
        for nd in G.nodes():
            if maxDist[nd] < best_val:
                best_val = maxDist[nd]
                best_node = nd
        
        path = []
        tmp = best_node
        while tmp is not None:
            path.append(tmp)
            tmp = pred[tmp]
        path.reverse()
        
        # Calculate branch statistics (do not modify function signature)
        branch_count = sum(1 for node in path if G.nodes[node].get('is_branch_point', False))
        
        if debug:
            print(f"[DEBUG] Longest path has {len(path)} nodes and {branch_count} branch points")
        
        return (best_node, path)



    def invert_graph_direction(self, G_in, cpoints):
        """
        Produce a new DiGraph G_out in which each edge is reversed. 
        Copy node/edge attributes.

        Parameters
        ----------
        G_in : nx.DiGraph
            The input directed graph.
        cpoints : np.ndarray
            Unused except if you wanted to condition on node positions.

        Returns
        -------
        G_out : nx.DiGraph
            A new graph with reversed edges, preserving node attributes.
        """
        G_out= nx.DiGraph()
        for nd, attr in G_in.nodes(data=True):
            G_out.add_node(nd, **attr)
        for (u,v) in G_in.edges():
            edata = G_in[u][v]
            G_out.add_edge(v, u, **edata)
        return G_out

    ##########################################################################
    # STEP 4: Labeling & Refinement
    ##########################################################################

    def label_main_stem_and_leaves(self, G, trunk_path):
        """
        Mark trunk path nodes as 'stem', everything else as 'leaf'.

        Parameters
        ----------
        G : nx.DiGraph or nx.Graph
            The graph to label in-place.
        trunk_path : list of int
            The node IDs that form the trunk path.
        """
        stem_set= set(trunk_path)
        for nd in G.nodes():
            if nd in stem_set:
                G.nodes[nd]["type"] = "stem"
            else:
                G.nodes[nd]["type"] = "leaf"

    def label_branch_off_nodes(self, G, min_degree=3, debug=True):
        """
        Mark any node with degree >= min_degree as 'branch_off'.
        Uses undirected degree calculation for consistency with branch point detection.
        
        Parameters
        ----------
        G : nx.Graph or nx.DiGraph
            The graph in which to label nodes.
        min_degree : int
            Degree threshold for labeling branch_off.
        debug : bool
            Whether to print debug information.
        """
        branch_points = []
        
        # Create undirected version for degree calculation (like in build_raindrop_negcost_digraph)
        undirected_G = nx.Graph()
        for nd in G.nodes():
            undirected_G.add_node(nd)
        
        # Add undirected edges
        for u, v in G.edges():
            undirected_G.add_edge(u, v)
        
        # Debug: Check degree distribution
        if debug:
            degrees = [undirected_G.degree(nd) for nd in G.nodes()]
            max_deg = max(degrees) if degrees else 0
            print(f"[DEBUG] label_branch_off_nodes: max_degree={max_deg}, min_degree_threshold={min_degree}")
            degree_counts = {}
            for deg in degrees:
                degree_counts[deg] = degree_counts.get(deg, 0) + 1
            print(f"[DEBUG] Degree distribution: {dict(sorted(degree_counts.items()))}")
        
        # Use undirected degree for branch point detection
        for nd in G.nodes():
            undirected_degree = undirected_G.degree(nd)
            if undirected_degree >= min_degree:
                G.nodes[nd]['type'] = 'branch_off'
                G.nodes[nd]['branch_count'] = undirected_degree - 2
                branch_points.append(nd)
                if debug:
                    print(f"[DEBUG] Found branch-off node {nd} with degree {undirected_degree}")
        
        if debug:
            print(f"[DEBUG] Total branch-off nodes found: {len(branch_points)}")
        
        return branch_points

    def refine_branch_off_nodes_immediate(self, G, min_stem=2, min_leaf=1, debug=False):
        """
        For each 'branch_off', check immediate neighbors. 
        If fewer than (min_stem) stem neighbors or (min_leaf) leaf neighbors, 
        re-label based on majority. 
        """
        def undirected_neighbors(u):
            return set(G.successors(u)) | set(G.predecessors(u))

        new_labels = {}
        branch_off_nodes = [nd for nd in G.nodes() if G.nodes[nd].get('type')=='branch_off']
        
        if debug:
            print(f"[DEBUG] refine_branch_off_nodes_immediate: processing {len(branch_off_nodes)} nodes")
            print(f"[DEBUG] Requirements: min_stem={min_stem}, min_leaf={min_leaf}")
        
        for nd in branch_off_nodes:
            nbrs = undirected_neighbors(nd)
            stem_count = sum(G.nodes[x].get('type')=='stem' for x in nbrs)
            leaf_count = sum(G.nodes[x].get('type')=='leaf' for x in nbrs)
            
            if debug:
                neighbor_types = [G.nodes[x].get('type') for x in nbrs]
                print(f"[DEBUG] Node {nd}: stem_neighbors={stem_count}, leaf_neighbors={leaf_count}, all_neighbors={neighbor_types}")
            
            if stem_count < min_stem or leaf_count < min_leaf:
                if stem_count > leaf_count:
                    new_labels[nd] = 'stem'
                elif leaf_count > stem_count:
                    new_labels[nd] = 'leaf'
                else:
                    new_labels[nd] = 'outlier'
                    
                if debug:
                    print(f"[DEBUG] Node {nd} relabeled from 'branch_off' to '{new_labels[nd]}'")
        
        for k, v in new_labels.items():
            G.nodes[k]['type'] = v
            
        if debug:
            print(f"[DEBUG] Relabeled {len(new_labels)} nodes")

    def refine_close_branch_off_nodes(self, G, trunk_path, branch_off_nodes,
                                      trunk_distance_threshold=4,
                                      leaf_overlap_threshold=0.5):
        """
        If two branch_off nodes are close along trunk_path (distance <= trunk_distance_threshold), 
        compare their sets of reachable leaf nodes. If overlap ratio >= leaf_overlap_threshold, 
        we unify by relabeling one as 'stem'.

        Parameters
        ----------
        G : nx.Graph or nx.DiGraph
            The graph (with 'type' attributes).
        trunk_path : list of int
            The trunk path (ordered).
        branch_off_nodes : list of int
            The node IDs labeled 'branch_off'.
        trunk_distance_threshold : int
            If the trunk-index distance is <= this, we compare them.
        leaf_overlap_threshold : float
            If overlap ratio >= this, we unify (relabel) one as 'stem'.
        """
        def undirected_neighbors(u):
            return set(G.successors(u)) | set(G.predecessors(u))

        from collections import deque

        def get_leaf_set(b_off):
            visited= set()
            queue = deque()
            leaf_nodes= set()
            for nb in undirected_neighbors(b_off):
                if G.nodes[nb].get('type')=='leaf':
                    queue.append(nb)
            while queue:
                curr= queue.popleft()
                if curr in visited:
                    continue
                visited.add(curr)
                if G.nodes[curr].get('type')=='leaf':
                    leaf_nodes.add(curr)
                    for nxt in undirected_neighbors(curr):
                        if nxt not in visited and nxt!=b_off and G.nodes[nxt].get('type')=='leaf':
                            queue.append(nxt)
            return leaf_nodes

        trunk_index_map= { nd:i for i,nd in enumerate(trunk_path) }
        branch_info={}
        for b_off in branch_off_nodes:
            idx= trunk_index_map.get(b_off, None)
            if idx is not None:
                lf_set= get_leaf_set(b_off)
                branch_info[b_off]= (idx, lf_set)

        sorted_boffs= sorted(branch_info.keys(), key=lambda x: branch_info[x][0])
        to_relabel= set()

        for i in range(len(sorted_boffs)):
            b1= sorted_boffs[i]
            if b1 in to_relabel:
                continue
            idx1, leafset1= branch_info[b1]
            for j in range(i+1, len(sorted_boffs)):
                b2= sorted_boffs[j]
                if b2 in to_relabel:
                    continue
                idx2, leafset2= branch_info[b2]
                dist_on_trunk= abs(idx2- idx1)
                if dist_on_trunk<= trunk_distance_threshold:
                    union_ = leafset1.union(leafset2)
                    inter_ = leafset1.intersection(leafset2)
                    if len(union_)>0:
                        overlap= len(inter_)/ len(union_)
                    else:
                        overlap= 0.0
                    if overlap>= leaf_overlap_threshold:
                        to_relabel.add(b2)
        for nd in to_relabel:
            G.nodes[nd]['type']= 'stem'

    ##########################################################################
    # STEP 5: Extract Branch Sections => Angles (Node-Based)
    ##########################################################################

    def extract_branch_sections_with_angles_node_based(
        self, G, cpoints, trunk_path, branch_off_nodes,
        n_main_stem=5,
        n_leaf=5,
        flip_if_obtuse=True,
        min_leaf_for_angle=5,
        max_bfs_depth=5
    ):
        """
        Node-based BFS approach to gather trunk coords near each branch_off => gather leaf => compute angle.

        Parameters
        ----------
        G : nx.Graph or nx.DiGraph
            The final labeled graph.
        cpoints : np.ndarray
            Node coordinates (N,3), node_id => cpoints[node_id].
        trunk_path : list of int
            The IDs in the main stem trunk path.
        branch_off_nodes : list of int
            The node IDs labeled 'branch_off'.
        n_main_stem : int
            How many trunk nodes to gather above/below the branch_off.
        n_leaf : int
            Max leaf nodes to collect by BFS.
        flip_if_obtuse : bool
            If True, angles >90 deg => 180 - angle.
        min_leaf_for_angle : int
            If BFS finds fewer leaves => skip angle.
        max_bfs_depth : int
            BFS limit for counting leaves.

        Returns
        -------
        branch_data : list of dict
            Each dict has keys:
             'branch_off' : node_id,
             'stem_points': Nx3 array,
             'leaf_points': Mx3 array,
             'angle_degrees': float
        """

        def undirected_neighbors(u):
            return set(G.successors(u)) | set(G.predecessors(u))

        def count_leaf_nodes_bfs(start, depth_limit):
            visited= set()
            queue  = deque([(start,0)])
            leaf_cnt= 0
            while queue:
                nd, dpt = queue.popleft()
                if nd in visited:
                    continue
                visited.add(nd)
                if G.nodes[nd].get('type')=='leaf':
                    leaf_cnt+=1
                if dpt< depth_limit:
                    for nb in undirected_neighbors(nd):
                        if nb not in visited:
                            queue.append((nb, dpt+1))
            return leaf_cnt

        def collect_leaf_nodes_bfs(start, n_leaf):
            visited= set()
            queue  = deque()
            leaves = []
            # Start from immediate leaf neighbors
            for nb in undirected_neighbors(start):
                if G.nodes[nb].get('type')=='leaf':
                    queue.append(nb)

            while queue and len(leaves)< n_leaf:
                curr= queue.popleft()
                if curr in visited:
                    continue
                visited.add(curr)
                if G.nodes[curr].get('type')=='leaf':
                    leaves.append(curr)
                    # expand neighbors
                    for nxt in undirected_neighbors(curr):
                        if nxt not in visited and G.nodes[nxt].get('type')=='leaf':
                            queue.append(nxt)
            return leaves

        def fit_line_svd(pts):
            arr= np.asarray(pts)
            if arr.shape[0]<2:
                return (None,None)
            center= arr.mean(axis=0)
            uu, ss, vh= np.linalg.svd(arr - center)
            direction= vh[0]/ np.linalg.norm(vh[0])
            return (center, direction)

        data_out = []
        trunk_set= set(trunk_path)

        for b_off in branch_off_nodes:
            if b_off not in trunk_set:
                print(f"[WARN] skip b_off= {b_off}, not trunk.")
                continue

            leaf_cnt= count_leaf_nodes_bfs(b_off, max_bfs_depth)
            if leaf_cnt< min_leaf_for_angle:
                print(f"[SKIP] b_off= {b_off}: only {leaf_cnt} leaves => skip angle.")
                continue

            idx= trunk_path.index(b_off)
            above_ids= trunk_path[idx+1: idx+1+n_main_stem]
            below_ids= trunk_path[max(0, idx-n_main_stem): idx]
            trunk_nodes= above_ids+ below_ids
            trunk_points= [cpoints[tn] for tn in trunk_nodes if tn< len(cpoints)]
            if len(trunk_points)<2:
                continue

            leaf_nodes= collect_leaf_nodes_bfs(b_off, n_leaf)
            leaf_points= [cpoints[ln] for ln in leaf_nodes if ln< len(cpoints)]
            if len(leaf_points)<2:
                continue

            # Fit lines
            stem_center, stem_dir= fit_line_svd(trunk_points)
            leaf_center, leaf_dir= fit_line_svd(leaf_points)
            if stem_dir is None or leaf_dir is None:
                continue

            dot_val= np.clip(np.dot(stem_dir, leaf_dir),-1,1)
            angle_deg= np.degrees(np.arccos(dot_val))
            if flip_if_obtuse and angle_deg>90:
                angle_deg= 180 - angle_deg

            data_out.append({
                'branch_off':   b_off,
                'stem_points':  np.array(trunk_points),
                'leaf_points':  np.array(leaf_points),
                'angle_degrees': angle_deg
            })
            print(f"[NODE-BASED-ANGLE] b_off= {b_off}, angle= {angle_deg:.2f}")

        return data_out

    ##########################################################################
    # STEP 6: Map Labels => Original Points
    ##########################################################################

    def map_labels_to_original_points_unified(self, G, slice_results, aggregated_map, original_pcd, output_path=None, base=None):
        """
        Modified to include all original points, filling gaps via nearest-neighbor label propagation.
        """
        color_map = {'stem': [1,0,0], 'leaf': [0,1,0], 'unknown': [0.6,0.6,0.6]}
        label_map = {'stem': 0, 'leaf': 1, 'unknown': 2}

        # Step 1: Generate initial labeled data from graph and slice_results
        cluster_to_nodes = defaultdict(list)
        for node_id, (si, cj) in enumerate(aggregated_map):
            cluster_to_nodes[(si, cj)].append(node_id)

        labeled_pts = []
        labeled_cols = []
        labeled_arr = []

        for si, slice in enumerate(slice_results):
            for cj, cluster in enumerate(slice):
                nodes = cluster_to_nodes.get((si, cj), [])
                types = []
                for nd in nodes:
                    t = G.nodes[nd].get('type', 'unknown')
                    if t == 'branch_off':
                        t = 'stem'
                    types.append(t)
                final_type = 'stem' if 'stem' in types else 'leaf' if 'leaf' in types else 'unknown'
                col = color_map[final_type]
                lbl = label_map[final_type]
                for p in cluster['points']:
                    labeled_pts.append(p)
                    labeled_cols.append(col)
                    labeled_arr.append([p[0], p[1], p[2], lbl])
        labeled_arr = np.array(labeled_arr, dtype=float)
        # Step 2: Identify missing points in the original_pcd and propagate labels
        original_points = np.asarray(original_pcd.points)
        if len(labeled_pts) == 0:
            # Edge case: No labels found; mark all as unknown
            filled_labels = np.full((len(original_points), 1), 2)
        else:
            # Build KDTree from initially labeled points
            labeled_pcd = o3d.geometry.PointCloud()
            labeled_pcd.points = o3d.utility.Vector3dVector(np.array(labeled_pts))
            tree = o3d.geometry.KDTreeFlann(labeled_pcd)

            filled_labels = []
            radius = 1e-6  # Adjust based on point cloud density (e.g., 1mm for real-world data)
            for p in original_points:
                # Check if the point is already labeled (exact match)
                [k, idx, _] = tree.search_radius_vector_3d(p, radius)
                if k > 0:
                    lbl = labeled_arr[idx[0], 3]
                else:
                    # Find nearest neighbor and inherit label
                    [k, idx, _] = tree.search_knn_vector_3d(p, 1)
                    lbl = labeled_arr[idx[0], 3] if k > 0 else 2
                filled_labels.append(lbl)
            filled_labels = np.array(filled_labels)

        # Step 3: Create output with ALL original points
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = original_pcd.points
        out_colors = np.array([color_map['stem' if lbl == 0 else 'leaf' if lbl == 1 else 'unknown'] 
                            for lbl in filled_labels])
        out_pcd.colors = o3d.utility.Vector3dVector(out_colors)
        labeled_arr = np.hstack([original_points, filled_labels.reshape(-1, 1)])

        points = np.asarray(original_pcd.points)
        colors = np.asarray(original_pcd.colors)
        labels = filled_labels.astype(np.uint8)
        if output_path:
            np.savez(
                os.path.join(output_path, base),
                points=points,        # XYZ coordinates (N,3)
                colors=colors,        # RGB colors (N,3)
                labels=labels         # Class labels (N,) 0=stem, 1=leaf, 2=unknown
            )
            print(f"Saved labeled data to {output_path} with {len(points)} points")
            
        return out_pcd, labeled_arr




    ##########################################################################
    # ------------------- INTERNAL BRIDGING METHODS -------------------------
    ##########################################################################

    def _bridge_subgraphs_full_increasing_debug(
        self,
        adjacency,
        node_map,
        cpoints,
        initial_dist=0.05,
        dist_step=0.01,
        max_dist=0.3,
        max_passes_each=3,
        debug=True
    ):
        """
        Iteratively unify subgraphs with bridging distance from initial_dist up to max_dist.
        Calls _bridge_subgraphs_full_debug internally.
        """
        bridging_dist= initial_dist
        final_subg= None

        while bridging_dist <= max_dist:
            if debug:
                print(f"[_bridge_subgraphs_full_increasing_debug] bridging_dist= {bridging_dist:.3f}")

            subg_list= self._bridge_subgraphs_full_debug(
                adjacency= adjacency,
                node_map= node_map,
                cpoints= cpoints,
                bridging_dist= bridging_dist,
                max_passes= max_passes_each,
                debug= debug
            )
            if len(subg_list)<=1:
                if debug:
                    print(f"[_bridge_subgraphs_full_increasing_debug] => single subgraph at dist={bridging_dist:.3f}")
                return subg_list, bridging_dist

            bridging_dist+= dist_step
            final_subg= subg_list

        if debug and final_subg:
            print(f"[_bridge_subgraphs_full_increasing_debug] Reached bridging_dist={bridging_dist:.3f} > max_dist => {len(final_subg)} subgraphs remain.")
        return final_subg, bridging_dist

    def _bridge_subgraphs_full_debug(
        self,
        adjacency,
        node_map,
        cpoints,
        bridging_dist=0.05,
        max_passes=5,
        debug=True
    ):
        """
        Repeatedly call _bridge_subgraphs_once_debug until no changes or fully connected.
        """
        for pass_num in range(max_passes):
            if debug:
                print(f"[_bridge_subgraphs_full_debug] pass={pass_num}")
            subg_list, changed= self._bridge_subgraphs_once_debug(
                adjacency,
                node_map,
                cpoints,
                bridging_dist= bridging_dist,
                debug= debug
            )
            if not changed:
                if debug:
                    print(f"[_bridge_subgraphs_full_debug] pass={pass_num} => no more changes => stop.")
                return subg_list
            if len(subg_list)<=1:
                if debug:
                    print(f"[_bridge_subgraphs_full_debug] pass={pass_num} => now fully connected.")
                return subg_list

        if debug:
            print(f"[_bridge_subgraphs_full_debug] Reached max_passes={max_passes}, still have {len(subg_list)} subgraphs.")
        return subg_list

    def _bridge_subgraphs_once_debug(
        self,
        adjacency,
        node_map,
        cpoints,
        bridging_dist=0.05,
        debug=True
    ):
        """
        One pass bridging: find disconnected subgraphs, unify those w/ dist <= bridging_dist.
        """
        subg_list= self._find_subgraphs_debug(adjacency, debug=debug)
        n_comps= len(subg_list)
        if n_comps<=1:
            if debug:
                print(f"[_bridge_subgraphs_once_debug] Only {n_comps} subgraph => no bridging needed.")
            return subg_list, False

        if debug:
            print(f"[_bridge_subgraphs_once_debug] Found {n_comps} subgraphs => bridgingDist={bridging_dist:.3f}")

        # build subgraph_id
        subgraph_id= {}
        for i, comp in enumerate(subg_list):
            for nd in comp:
                subgraph_id[nd]= i

        # gather bridging candidates
        bridging_candidates= []
        for i in range(n_comps):
            for j in range(i+1, n_comps):
                dist_val, (ndA, ndB)= self._find_closest_pair_between_subgraphs_debug(
                    subg_list[i], subg_list[j],
                    node_map, cpoints, debug=False
                )
                bridging_candidates.append((dist_val, ndA, ndB, i, j))

        bridging_candidates.sort(key= lambda x: x[0])

        changed= False
        for (dist_val, ndA, ndB, iA, iB) in bridging_candidates:
            if dist_val> bridging_dist:
                break
            if subgraph_id[ndA]== subgraph_id[ndB]:
                continue
            if debug:
                print(f"[_bridge_subgraphs_once_debug] bridging subgraph {subgraph_id[ndA]} & {subgraph_id[ndB]} via nodes {ndA}--{ndB}, dist={dist_val:.3f}")

            self._safe_append_edge(adjacency, ndA, ndB)
            changed= True

            # optional vertical adjacency
            i_slice, i_cluster= ndA
            k_slice, k_cluster= ndB
            if k_slice== i_slice+1:
                if debug:
                    print(f" => Also adding vertical adjacency {ndA} -> {ndB}")
                if ndB not in adjacency[ndA]:
                    adjacency[ndA].append(ndB)
            elif i_slice== k_slice+1:
                if debug:
                    print(f" => Also adding vertical adjacency {ndB} -> {ndA}")
                if ndA not in adjacency[ndB]:
                    adjacency[ndB].append(ndA)

            # unify subgraphs
            oldID= subgraph_id[ndB]
            newID= subgraph_id[ndA]
            for ndX in subg_list[oldID]:
                subgraph_id[ndX]= newID
            subg_list[newID].update(subg_list[oldID])
            subg_list[oldID].clear()

        # reassemble ignoring empties
        final_subg= []
        for comp in subg_list:
            if len(comp)>0:
                final_subg.append(comp)

        return final_subg, changed

    def _safe_append_edge(self, adjacency, ndA, ndB):
        """Ensure adjacency[ndA], adjacency[ndB] exist, then link them both ways."""
        if ndA not in adjacency:
            adjacency[ndA]= []
        if ndB not in adjacency:
            adjacency[ndB]= []
        adjacency[ndA].append(ndB)
        adjacency[ndB].append(ndA)

    def _find_subgraphs_debug(self, adjacency, debug=True):
        """
        BFS subgraph detection with debug prints.
        """
        visited= set()
        subgraphs= []
        all_nodes= list(adjacency.keys())
        if debug:
            print(f"[_find_subgraphs_debug] total nodes in adjacency= {len(all_nodes)}")

        for node in all_nodes:
            if node not in visited:
                comp= set()
                queue= deque([node])
                visited.add(node)
                if debug:
                    print(f" => BFS start node= {node}")
                while queue:
                    curr= queue.popleft()
                    comp.add(curr)
                    neighbors= adjacency.get(curr, [])
                    for nb in neighbors:
                        if nb not in visited:
                            visited.add(nb)
                            queue.append(nb)
                subgraphs.append(comp)
                if debug:
                    print(f" => subgraph found => size= {len(comp)}")

        if debug:
            print(f" => total subgraphs= {len(subgraphs)}")
        return subgraphs
    
    def _bridge_subgraphs_with_geometric_constraints(
        self,
        adjacency,
        node_map,
        cpoints,
        initial_dist=0.05,
        dist_step=0.01,
        max_dist=0.3,
        max_angle_deg=45,
        max_z_gap=3,
        density_tolerance=2.0,
        debug=True
    ):
        """
        Enhanced bridging with geometric constraints to prevent unrealistic connections.
        
        This method improves upon basic bridging by:
        1. Adding angle constraints to prevent unrealistic bends
        2. Limiting Z-level gaps for more natural connections
        3. Considering density consistency between regions
        4. Validating structural plausibility of connections
        
        Parameters
        ----------
        adjacency : dict
            The adjacency dictionary to modify.
        node_map : dict
            Mapping from (slice_i, cluster_j) to node_id.
        cpoints : np.ndarray
            Centroid coordinates for all nodes.
        initial_dist : float, default=0.05
            Starting distance threshold for bridging.
        dist_step : float, default=0.01
            Step size for increasing distance threshold.
        max_dist : float, default=0.3
            Maximum distance threshold.
        max_angle_deg : float, default=45
            Maximum angle deviation from vertical (degrees).
        max_z_gap : int, default=3
            Maximum Z-level gap for bridging.
        density_tolerance : float, default=2.0
            Maximum density ratio between connected regions.
        debug : bool, default=True
            Whether to print debugging information.
            
        Returns
        -------
        final_subgraphs : list
            List of connected subgraph components.
        used_distance : float
            The final distance threshold used.
        bridging_stats : dict
            Statistics about the bridging process.
        """
        bridging_stats = {
            'total_attempts': 0,
            'successful_bridges': 0,
            'rejected_angle': 0,
            'rejected_z_gap': 0,
            'rejected_density': 0,
            'rejected_structural': 0,
            'distance_used': initial_dist
        }
        
        bridging_dist = initial_dist
        final_subg = None
        
        while bridging_dist <= max_dist:
            if debug:
                print(f"[GEOMETRIC-BRIDGE] Trying bridging_dist={bridging_dist:.3f}")
            
            subg_list, iteration_stats = self._bridge_subgraphs_once_geometric(
                adjacency, node_map, cpoints, bridging_dist,
                max_angle_deg, max_z_gap, density_tolerance, debug
            )
            
            # Update cumulative stats
            for key in iteration_stats:
                if key in bridging_stats:
                    bridging_stats[key] += iteration_stats[key]
            
            if len(subg_list) <= 1:
                if debug:
                    print(f"[GEOMETRIC-BRIDGE] Achieved single subgraph at dist={bridging_dist:.3f}")
                bridging_stats['distance_used'] = bridging_dist
                return subg_list, bridging_dist, bridging_stats
            
            bridging_dist += dist_step
            final_subg = subg_list
        
        if debug and final_subg:
            print(f"[GEOMETRIC-BRIDGE] Reached max_dist={max_dist:.3f}, {len(final_subg)} subgraphs remain")
            print(f"[GEOMETRIC-BRIDGE] Stats: {bridging_stats['successful_bridges']} successful, "
                  f"{bridging_stats['rejected_angle']} angle rejected, "
                  f"{bridging_stats['rejected_z_gap']} z-gap rejected, "
                  f"{bridging_stats['rejected_density']} density rejected")
        
        bridging_stats['distance_used'] = max_dist
        return final_subg or [], max_dist, bridging_stats
    
    def _bridge_subgraphs_once_geometric(
        self,
        adjacency,
        node_map,
        cpoints,
        bridging_dist,
        max_angle_deg,
        max_z_gap,
        density_tolerance,
        debug=False
    ):
        """
        Single pass of geometric bridging with multiple constraint checks.
        """
        stats = {
            'total_attempts': 0,
            'successful_bridges': 0,
            'rejected_angle': 0,
            'rejected_z_gap': 0,
            'rejected_density': 0,
            'rejected_structural': 0
        }
        
        subg_list = self._find_subgraphs_debug(adjacency, debug=False)
        n_comps = len(subg_list)
        
        if n_comps <= 1:
            return subg_list, stats
        
        # Build subgraph mapping and analyze subgraph properties
        subgraph_id = {}
        subgraph_properties = {}
        
        for i, comp in enumerate(subg_list):
            # Map nodes to subgraph ID
            for nd in comp:
                subgraph_id[nd] = i
            
            # Analyze subgraph properties
            subgraph_properties[i] = self._analyze_subgraph_properties(comp, node_map, cpoints)
        
        # Find valid bridging candidates with geometric constraints
        bridging_candidates = []
        
        for i in range(n_comps):
            for j in range(i + 1, n_comps):
                stats['total_attempts'] += 1
                
                # Find closest pair between subgraphs
                dist_val, (ndA, ndB) = self._find_closest_pair_between_subgraphs_debug(
                    subg_list[i], subg_list[j], node_map, cpoints, debug=False
                )
                
                if dist_val > bridging_dist:
                    continue
                
                # Apply geometric constraints
                constraint_result = self._check_bridging_constraints(
                    ndA, ndB, node_map, cpoints, subgraph_properties[i], subgraph_properties[j],
                    max_angle_deg, max_z_gap, density_tolerance
                )
                
                if constraint_result['valid']:
                    bridging_candidates.append((
                        constraint_result['weighted_cost'], dist_val, ndA, ndB, i, j
                    ))
                else:
                    # Track rejection reasons
                    for reason in constraint_result['rejection_reasons']:
                        stats[f'rejected_{reason}'] += 1
        
        # Sort by weighted cost (lower is better)
        bridging_candidates.sort(key=lambda x: x[0])
        
        # Apply bridges greedily
        changed = False
        for (weighted_cost, dist_val, ndA, ndB, iA, iB) in bridging_candidates:
            # Check if these subgraphs are still separate
            if subgraph_id[ndA] == subgraph_id[ndB]:
                continue
            
            if debug:
                print(f"[GEOMETRIC-BRIDGE] Bridging subgraphs {subgraph_id[ndA]} & {subgraph_id[ndB]} "
                      f"via {ndA}--{ndB}, dist={dist_val:.3f}, cost={weighted_cost:.3f}")
            
            # Create the bridge
            self._safe_append_edge(adjacency, ndA, ndB)
            changed = True
            stats['successful_bridges'] += 1
            
            # Add vertical adjacency if appropriate
            self._add_vertical_adjacency_if_valid(adjacency, ndA, ndB, debug)
            
            # Update subgraph membership
            oldID = subgraph_id[ndB]
            newID = subgraph_id[ndA]
            for ndX in subg_list[oldID]:
                subgraph_id[ndX] = newID
            subg_list[newID].update(subg_list[oldID])
            subg_list[oldID].clear()
        
        # Reassemble non-empty subgraphs
        final_subg = [comp for comp in subg_list if len(comp) > 0]
        
        return final_subg, stats
    
    def _analyze_subgraph_properties(self, subgraph_nodes, node_map, cpoints):
        """
        Analyze geometric and density properties of a subgraph component.
        """
        if not subgraph_nodes:
            return {'density': 0, 'z_range': (0, 0), 'centroid': np.zeros(3), 'size': 0}
        
        # Get all points in this subgraph
        node_positions = []
        z_levels = []
        
        for node_key in subgraph_nodes:
            if node_key in node_map:
                node_id = node_map[node_key]
                if node_id < len(cpoints):
                    pos = cpoints[node_id]
                    node_positions.append(pos)
                    z_levels.append(pos[2])
            else:
                # Handle case where node_key is already a node_id
                if isinstance(node_key, tuple):
                    slice_idx, cluster_idx = node_key
                    z_levels.append(slice_idx)  # Approximate Z from slice index
                
        if not node_positions:
            return {'density': 0, 'z_range': (0, 0), 'centroid': np.zeros(3), 'size': 0}
        
        positions = np.array(node_positions)
        z_values = np.array(z_levels)
        
        # Calculate properties
        centroid = positions.mean(axis=0)
        z_min, z_max = z_values.min(), z_values.max()
        z_range = (z_min, z_max)
        
        # Estimate density (points per unit volume)
        if len(positions) > 1:
            bbox_volume = np.prod(positions.max(axis=0) - positions.min(axis=0) + 1e-6)
            density = len(positions) / bbox_volume
        else:
            density = 1.0
        
        return {
            'density': density,
            'z_range': z_range,
            'centroid': centroid,
            'size': len(subgraph_nodes),
            'positions': positions
        }
    
    def _check_bridging_constraints(
        self, ndA, ndB, node_map, cpoints, props_A, props_B,
        max_angle_deg, max_z_gap, density_tolerance
    ):
        """
        Check if a potential bridge satisfies all geometric constraints.
        """
        result = {
            'valid': True,
            'rejection_reasons': [],
            'weighted_cost': 0.0
        }
        
        # Get node positions
        node_id_A = node_map.get(ndA, ndA if isinstance(ndA, int) else -1)
        node_id_B = node_map.get(ndB, ndB if isinstance(ndB, int) else -1)
        
        if node_id_A >= len(cpoints) or node_id_B >= len(cpoints) or node_id_A < 0 or node_id_B < 0:
            result['valid'] = False
            result['rejection_reasons'].append('structural')
            return result
        
        pos_A = cpoints[node_id_A]
        pos_B = cpoints[node_id_B]
        bridge_vector = pos_B - pos_A
        bridge_distance = np.linalg.norm(bridge_vector)
        
        if bridge_distance < 1e-6:
            result['valid'] = False
            result['rejection_reasons'].append('structural')
            return result
        
        # 1. Angle constraint - check deviation from vertical
        vertical_vector = np.array([0, 0, 1])
        if bridge_distance > 1e-6:
            bridge_direction = bridge_vector / bridge_distance
            angle_from_vertical = np.arccos(np.clip(
                np.abs(np.dot(bridge_direction, vertical_vector)), 0, 1
            )) * 180 / np.pi
            
            if angle_from_vertical > max_angle_deg:
                result['valid'] = False
                result['rejection_reasons'].append('angle')
        
        # 2. Z-gap constraint
        z_gap = abs(pos_A[2] - pos_B[2])
        if isinstance(ndA, tuple) and isinstance(ndB, tuple):
            slice_gap = abs(ndA[0] - ndB[0])  # Z difference in slice indices
            if slice_gap > max_z_gap:
                result['valid'] = False
                result['rejection_reasons'].append('z_gap')
        
        # 3. Density consistency constraint
        density_A = props_A.get('density', 1.0)
        density_B = props_B.get('density', 1.0)
        
        if density_A > 0 and density_B > 0:
            density_ratio = max(density_A, density_B) / min(density_A, density_B)
            if density_ratio > density_tolerance:
                result['valid'] = False
                result['rejection_reasons'].append('density')
        
        # 4. Structural plausibility - check for reasonable geometric arrangement
        centroid_A = props_A.get('centroid', pos_A)
        centroid_B = props_B.get('centroid', pos_B)
        
        # Distance between centroids should be reasonable compared to bridge distance
        centroid_distance = np.linalg.norm(centroid_B - centroid_A)
        if centroid_distance > 0:
            bridge_efficiency = bridge_distance / centroid_distance
            if bridge_efficiency > 2.0:  # Bridge is much longer than direct path
                result['valid'] = False
                result['rejection_reasons'].append('structural')
        
        # Calculate weighted cost for valid bridges (lower is better)
        if result['valid']:
            # Base cost is the bridge distance
            base_cost = bridge_distance
            
            # Add penalty for angle deviation
            angle_penalty = (angle_from_vertical / max_angle_deg) * 0.5
            
            # Add penalty for Z-gap
            z_gap_penalty = (z_gap / max_z_gap) * 0.3 if max_z_gap > 0 else 0
            
            # Add penalty for density mismatch
            density_penalty = (density_ratio - 1) / density_tolerance * 0.2 if density_tolerance > 1 else 0
            
            result['weighted_cost'] = base_cost * (1 + angle_penalty + z_gap_penalty + density_penalty)
        
        return result
    
    def _add_vertical_adjacency_if_valid(self, adjacency, ndA, ndB, debug=False):
        """
        Add vertical adjacency between nodes if they are in consecutive slices.
        """
        if not isinstance(ndA, tuple) or not isinstance(ndB, tuple):
            return
        
        i_slice, i_cluster = ndA
        k_slice, k_cluster = ndB
        
        # Only add if slices are consecutive
        if k_slice == i_slice + 1:
            if debug:
                print(f"  => Adding vertical adjacency {ndA} -> {ndB}")
            if ndB not in adjacency.get(ndA, []):
                adjacency.setdefault(ndA, []).append(ndB)
        elif i_slice == k_slice + 1:
            if debug:
                print(f"  => Adding vertical adjacency {ndB} -> {ndA}")
            if ndA not in adjacency.get(ndB, []):
                adjacency.setdefault(ndB, []).append(ndA)

    def _find_closest_pair_between_subgraphs_debug(self, subgA, subgB, node_map, cpoints, debug=False):
        """
        Minimal 3D distance among all pairs => (distVal, (ndA, ndB)).
        """
        best_dist= float('inf')
        best_pair= (None,None)
        for ndA in subgA:
            idxA= node_map[ndA]
            pA= cpoints[idxA]
            for ndB in subgB:
                idxB= node_map[ndB]
                pB= cpoints[idxB]
                d= np.linalg.norm(pA- pB)
                if d< best_dist:
                    best_dist= d
                    best_pair= (ndA, ndB)
        if debug:
            print(f" => find_closest_pair_between_subgraphs => dist={best_dist:.3f}, pair={best_pair}")
        return best_dist, best_pair



    ##########################################################################
    # MASTER RUN PIPELINE
    ##########################################################################


    def run_full_pipeline(
        self,
        alpha=1.0,
        beta=0.5,
        raindrop_alpha=1.0,
        raindrop_beta=1.0,
        gamma=2.0,
        delta=1.0,
        use_trunk_axis=True,
        debug=True,
        output_path=None,
        base=None
    ):
        """
        1) slicing & clustering
        2) build adjacency A & B
        3) bridging subgraphs
        4) build negative-cost DAG, trunk extraction
        5) label trunk => 'stem' & everything else => 'leaf'
        6) label_branch_off_nodes(min_degree=3)
            => now we see branch_off if degree≥3
        7) (optional) refine branch_off nodes
        8) compute angles
        9) map final labels => labeled_pcd
        """
        # ---------------------------------------------------------------
        # 1) Slicing + clustering if not already done
        # ---------------------------------------------------------------
        if not self.slices:
            self.slice_cloud_z(n_sections=None, max_slices=80, show_hist=False)
        if not self.slice_results:
            self.cluster_slices(
                base_scale=5.0, 
                min_samples=10, 
                dist_merge=0.03, 
                min_pts_in_cluster=15
            )

        # ---------------------------------------------------------------
        # 2) Compute trunk axis => used in adjacency
        # ---------------------------------------------------------------
        trunk_axis = self.compute_principal_axis()
        if debug:
            print("[DEBUG] trunk_axis =", trunk_axis)

        # ---------------------------------------------------------------
        # 3) Build adjacency (A + B), bridging, negative-cost DAG
        # ---------------------------------------------------------------

        ### --- adjacency A
        adjA = self.build_adjacency_bipartite(max_dist=0.1)
        GA, node_mapA, cpointsA = self.build_cost_graph(self.slice_results, adjA, trunk_axis, alpha=alpha)

        # bridging subgraphs => same as your script
        adjA_copy = copy.deepcopy(adjA)
        final_subgA, used_distA = self._bridge_subgraphs_full_increasing_debug(
            adjacency=adjA_copy,
            node_map=node_mapA,
            cpoints=cpointsA,
            debug=debug
        )
        if debug:
            print(f"[A] bridging => final bridging_dist= {used_distA:.3f}, subg= {len(final_subgA)}")

        GnegA = self.build_raindrop_negcost_digraph(
            cpointsA, adjA_copy, node_mapA,
            alpha=raindrop_alpha,
            beta=raindrop_beta,
            gamma=gamma, 
            delta=delta,
            trunk_axis=trunk_axis if use_trunk_axis else None,
            reverse_z=True,
            debug=debug
        )
        zA = cpointsA[:, 2]
        topA = int(zA.argmax())
        baseA, trunkA = self.longest_path_in_DAG_negcost(GnegA, topA, debug=debug)
        costA = 0.0
        for i2 in range(len(trunkA) - 1):
            costA += GnegA[trunkA[i2]][trunkA[i2+1]]['weight']
        if debug:
            print(f"[A] trunk => base={baseA}, top={topA}, cost={costA:.3f}")

        ### --- adjacency B
        adjB = self.build_adjacency_bipartite_vertical(
            trunk_axis, max_dist=1.5, alpha=alpha, beta=beta
        )
        GB, node_mapB, cpointsB = self.build_cost_graph(self.slice_results, adjB, trunk_axis, alpha=alpha)

        adjB_copy = copy.deepcopy(adjB)
        final_subgB, used_distB = self._bridge_subgraphs_full_increasing_debug(
            adjacency=adjB_copy,
            node_map=node_mapB,
            cpoints=cpointsB,
            debug=debug
        )
        if debug:
            print(f"[B] bridging => final bridging_dist= {used_distB:.3f}, subg= {len(final_subgB)}")

        GnegB = self.build_raindrop_negcost_digraph(
            cpointsB, adjB_copy, node_mapB,
            alpha=raindrop_alpha,
            beta=raindrop_beta,
            gamma=gamma, 
            delta=delta,
            trunk_axis=trunk_axis if use_trunk_axis else None,
            reverse_z=True,
            debug=debug
        )
        zB = cpointsB[:,2]
        topB = int(zB.argmax())
        baseB, trunkB = self.longest_path_in_DAG_negcost(GnegB, topB, debug=debug)
        costB = 0.0
        for i2 in range(len(trunkB) - 1):
            costB += GnegB[trunkB[i2]][trunkB[i2 + 1]]['weight']
        if debug:
            print(f"[B] trunk => base={baseB}, top={topB}, cost={costB:.3f}")

        # ---------------------------------------------------------------
        # 4) Choose whichever adjacency => trunk cost is more negative
        # ---------------------------------------------------------------
        if costA < costB:
            self.chosen_adjacency = adjA
            self.cpoints_final    = cpointsA
            self.node_map_final  = node_mapA
            self.G_neg_final     = GnegA
            self.trunk_path      = trunkA
            self.base_node       = baseA
            self.top_node        = topA
            if debug:
                print(f"[MASTER] chosen adjacency= A, trunk_cost= {costA:.3f}")
        else:
            self.chosen_adjacency = adjB
            self.cpoints_final    = cpointsB
            self.node_map_final  = node_mapB
            self.G_neg_final     = GnegB
            self.trunk_path      = trunkB
            self.base_node       = baseB
            self.top_node        = topB
            if debug:
                print(f"[MASTER] chosen adjacency= B, trunk_cost= {costB:.3f}")

        # Optionally invert direction
        self.G_neg_final = self.invert_graph_direction(self.G_neg_final, self.cpoints_final)

        # ---------------------------------------------------------------
        # 5) Label trunk => 'stem', others => 'leaf'
        # ---------------------------------------------------------------
        self.label_main_stem_and_leaves(self.G_neg_final, self.trunk_path)

        # 6) label_branch_off_nodes => now we get nodes with degree≥3 => 'branch_off'
        self.label_branch_off_nodes(self.G_neg_final, min_degree=3, debug=debug)

        # Build aggregated map => node_id => (slice_i, cluster_j)
        ag_map = [None]* len(self.cpoints_final)
        for (si,cj), nd in self.node_map_final.items():
            ag_map[nd] = (si,cj)
        self.aggregated_centroids_map = ag_map

        # 7) (optional) REFINE => if you still want outlier removal 
        # ---------------------------------------------------------------
        if debug:
            print("[DEBUG] => now refine outlier branch_off, if any.")

        self.refine_branch_off_nodes_immediate(self.G_neg_final, min_stem=2, min_leaf=1)
        boffs = [
            nd for nd in self.G_neg_final.nodes()
            if self.G_neg_final.nodes[nd].get('type') == 'branch_off'
        ]
        self.refine_close_branch_off_nodes(
            self.G_neg_final,
            self.trunk_path,
            boffs,
            trunk_distance_threshold=4,
            leaf_overlap_threshold=0.5
        )
        self.branch_off_nodes = [
            nd for nd in self.G_neg_final.nodes()
            if self.G_neg_final.nodes[nd].get('type')=='branch_off'
        ]

        # ---------------------------------------------------------------
        # 8) extract angles if you want node-based BFS => trunk vs. leaf
        # ---------------------------------------------------------------
        # self.branch_data = self.extract_branch_sections_with_angles_node_based(
        #     self.G_neg_final,
        #     self.cpoints_final,
        #     self.trunk_path,
        #     self.branch_off_nodes,
        #     n_main_stem=5,
        #     n_leaf=5,
        #     flip_if_obtuse=True,
        #     min_leaf_for_angle=4,
        #     max_bfs_depth=5
        # )


        # for bd in self.branch_data:
        #     # We only have one trunk group, treat as "up"
        #     up_pts = bd['stem_points']
        #     down_pts = np.empty((0,3))
        #     leaf_pts = bd['leaf_points']
        #     self.all_main_stem_points_up.append(up_pts)
        #     self.all_main_stem_points_down.append(down_pts)
        #     self.all_leaf_points.append(leaf_pts)





        # ---------------------------------------------------------------
        # 9) unify labels => original points => color-coded PCD
        # ---------------------------------------------------------------
        self.labeled_pcd, labeled_arr = self.map_labels_to_original_points_unified(
            self.G_neg_final,
            self.slice_results,
            self.aggregated_centroids_map,
            self.original_pcd,
            output_path=output_path,
            base=base

        )
        if debug:
            print(f"[INFO] Created labeled PCD => #points= {len(self.labeled_pcd.points)}")
            print("[END OF PIPELINE] => main stem & branch_off labeled => done.")

        return self.branch_data

    def run_adaptive_pipeline(
        self,
        use_adaptive_slicing=True,
        use_adaptive_clustering=True,
        use_geometric_bridging=True,
        # Adaptive slicing parameters
        target_points_per_slice=200,
        overlap_ratio=0.2,
        # Adaptive clustering parameters
        multi_scale_levels=3,
        density_factor=2.0,
        outlier_threshold=0.1,
        # Geometric bridging parameters
        max_angle_deg=45,
        max_z_gap=3,
        density_tolerance=2.0,
        # Standard pipeline parameters
        alpha=1.0,
        beta=0.5,
        raindrop_alpha=1.0,
        raindrop_beta=1.0,
        gamma=2.0,
        delta=1.0,
        use_trunk_axis=True,
        debug=True,
        output_path=None,
        base=None
    ):
        """
        Enhanced pipeline using adaptive slicing, multi-scale clustering, and geometric bridging.
        
        This method provides significant improvements for complex point cloud data by:
        1. Using adaptive slicing based on point density distribution
        2. Applying multi-scale DBSCAN with density-based eps adjustment
        3. Implementing geometric constraints in bridging to prevent unrealistic connections
        4. Better handling of outliers and noise
        
        Parameters
        ----------
        use_adaptive_slicing : bool, default=True
            Whether to use adaptive slicing instead of fixed slicing.
        use_adaptive_clustering : bool, default=True
            Whether to use multi-scale adaptive clustering.
        use_geometric_bridging : bool, default=True
            Whether to use geometric constraints in bridging.
        target_points_per_slice : int, default=200
            Target points per slice for adaptive slicing.
        overlap_ratio : float, default=0.2
            Overlap ratio between slices.
        multi_scale_levels : int, default=3
            Number of scales for multi-scale clustering.
        density_factor : float, default=2.0
            Density adaptation factor for clustering.
        outlier_threshold : float, default=0.1
            Threshold for outlier detection.
        max_angle_deg : float, default=45
            Maximum angle deviation for bridging.
        max_z_gap : int, default=3
            Maximum Z-level gap for bridging.
        density_tolerance : float, default=2.0
            Density tolerance for bridging.
        alpha, beta, gamma, delta : float
            Various cost function parameters.
        use_trunk_axis : bool, default=True
            Whether to use computed trunk axis.
        debug : bool, default=True
            Whether to print debugging information.
        output_path : str, optional
            Path to save results.
        base : str, optional
            Base filename for saving.
            
        Returns
        -------
        branch_data : list
            List of branch analysis results.
        pipeline_stats : dict
            Statistics about the enhanced pipeline performance.
        """
        pipeline_stats = {
            'adaptive_slicing_used': use_adaptive_slicing,
            'adaptive_clustering_used': use_adaptive_clustering,
            'geometric_bridging_used': use_geometric_bridging,
            'slicing_stats': {},
            'clustering_stats': {},
            'bridging_stats': {},
            'adjacency_chosen': None,
            'trunk_cost_A': None,
            'trunk_cost_B': None
        }
        
        if debug:
            print("[ADAPTIVE-PIPELINE] Starting enhanced pipeline for complex point clouds")
            print(f"  - Adaptive slicing: {use_adaptive_slicing}")
            print(f"  - Adaptive clustering: {use_adaptive_clustering}")
            print(f"  - Geometric bridging: {use_geometric_bridging}")
        
        # ---------------------------------------------------------------
        # 1) Enhanced Slicing
        # ---------------------------------------------------------------
        if not self.slices:
            if use_adaptive_slicing:
                if debug:
                    print("[ADAPTIVE-PIPELINE] Using adaptive slicing...")
                slices, slice_info = self.slice_cloud_z_adaptive(
                    target_points_per_slice=target_points_per_slice,
                    overlap_ratio=overlap_ratio,
                    min_slices=10,
                    max_slices=120,
                    show_hist=debug
                )
                pipeline_stats['slicing_stats'] = {
                    'method': 'adaptive',
                    'n_slices': len(slices),
                    'avg_points_per_slice': np.mean([len(s) for s in slices]),
                    'overlap_ratio': overlap_ratio
                }
            else:
                if debug:
                    print("[ADAPTIVE-PIPELINE] Using standard slicing...")
                self.slice_cloud_z(n_sections=None, max_slices=80, show_hist=False)
                pipeline_stats['slicing_stats'] = {
                    'method': 'standard',
                    'n_slices': self.n_slices
                }
        
        # ---------------------------------------------------------------
        # 2) Enhanced Clustering
        # ---------------------------------------------------------------
        if not self.slice_results:
            if use_adaptive_clustering:
                if debug:
                    print("[ADAPTIVE-PIPELINE] Using adaptive clustering...")
                slice_results, clustering_stats = self.cluster_slices_adaptive(
                    base_scale=5.0,
                    min_samples=15,
                    dist_merge=0.02,
                    min_pts_in_cluster=10,
                    density_factor=density_factor,
                    multi_scale_levels=multi_scale_levels,
                    outlier_threshold=outlier_threshold,
                    debug=debug
                )
                pipeline_stats['clustering_stats'] = clustering_stats
            else:
                if debug:
                    print("[ADAPTIVE-PIPELINE] Using standard clustering...")
                self.cluster_slices(
                    base_scale=5.0,
                    min_samples=10,
                    dist_merge=0.03,
                    min_pts_in_cluster=15
                )
                pipeline_stats['clustering_stats'] = {'method': 'standard'}
        
        # ---------------------------------------------------------------
        # 3) Compute trunk axis
        # ---------------------------------------------------------------
        trunk_axis = self.compute_principal_axis()
        if debug:
            print(f"[ADAPTIVE-PIPELINE] Computed trunk axis: {trunk_axis}")
        
        # ---------------------------------------------------------------
        # 4) Enhanced Adjacency Construction and Bridging
        # ---------------------------------------------------------------
        
        # Method A: Simple bipartite
        if debug:
            print("[ADAPTIVE-PIPELINE] Building adjacency A (bipartite)...")
        adjA = self.build_adjacency_bipartite(max_dist=0.1)
        GA, node_mapA, cpointsA = self.build_cost_graph(self.slice_results, adjA, trunk_axis, alpha=alpha)
        
        # Enhanced bridging for A
        adjA_copy = copy.deepcopy(adjA)
        if use_geometric_bridging:
            final_subgA, used_distA, bridging_statsA = self._bridge_subgraphs_with_geometric_constraints(
                adjacency=adjA_copy,
                node_map=node_mapA,
                cpoints=cpointsA,
                max_angle_deg=max_angle_deg,
                max_z_gap=max_z_gap,
                density_tolerance=density_tolerance,
                debug=debug
            )
            pipeline_stats['bridging_stats']['method_A'] = bridging_statsA
        else:
            final_subgA, used_distA = self._bridge_subgraphs_full_increasing_debug(
                adjacency=adjA_copy,
                node_map=node_mapA,
                cpoints=cpointsA,
                debug=debug
            )
        
        # Build negative-cost DAG for A
        GnegA = self.build_raindrop_negcost_digraph(
            cpointsA, adjA_copy, node_mapA,
            alpha=raindrop_alpha, beta=raindrop_beta, gamma=gamma, delta=delta,
            trunk_axis=trunk_axis if use_trunk_axis else None,
            reverse_z=True, debug=debug
        )
        zA = cpointsA[:, 2]
        topA = int(zA.argmax())
        baseA, trunkA = self.longest_path_in_DAG_negcost(GnegA, topA, debug=debug)
        costA = sum(GnegA[trunkA[i]][trunkA[i+1]]['weight'] for i in range(len(trunkA) - 1))
        pipeline_stats['trunk_cost_A'] = costA
        
        # Method B: Vertical bipartite
        if debug:
            print("[ADAPTIVE-PIPELINE] Building adjacency B (vertical)...")
        adjB = self.build_adjacency_bipartite_vertical(
            trunk_axis, max_dist=1.5, alpha=alpha, beta=beta
        )
        GB, node_mapB, cpointsB = self.build_cost_graph(self.slice_results, adjB, trunk_axis, alpha=alpha)
        
        # Enhanced bridging for B
        adjB_copy = copy.deepcopy(adjB)
        if use_geometric_bridging:
            final_subgB, used_distB, bridging_statsB = self._bridge_subgraphs_with_geometric_constraints(
                adjacency=adjB_copy,
                node_map=node_mapB,
                cpoints=cpointsB,
                max_angle_deg=max_angle_deg,
                max_z_gap=max_z_gap,
                density_tolerance=density_tolerance,
                debug=debug
            )
            pipeline_stats['bridging_stats']['method_B'] = bridging_statsB
        else:
            final_subgB, used_distB = self._bridge_subgraphs_full_increasing_debug(
                adjacency=adjB_copy,
                node_map=node_mapB,
                cpoints=cpointsB,
                debug=debug
            )
        
        # Build negative-cost DAG for B
        GnegB = self.build_raindrop_negcost_digraph(
            cpointsB, adjB_copy, node_mapB,
            alpha=raindrop_alpha, beta=raindrop_beta, gamma=gamma, delta=delta,
            trunk_axis=trunk_axis if use_trunk_axis else None,
            reverse_z=True, debug=debug
        )
        zB = cpointsB[:, 2]
        topB = int(zB.argmax())
        baseB, trunkB = self.longest_path_in_DAG_negcost(GnegB, topB, debug=debug)
        costB = sum(GnegB[trunkB[i]][trunkB[i+1]]['weight'] for i in range(len(trunkB) - 1))
        pipeline_stats['trunk_cost_B'] = costB
        
        # ---------------------------------------------------------------
        # 5) Choose best adjacency method
        # ---------------------------------------------------------------
        if costA < costB:
            self.chosen_adjacency = adjA
            self.cpoints_final = cpointsA
            self.node_map_final = node_mapA
            self.G_neg_final = GnegA
            self.trunk_path = trunkA
            self.base_node = baseA
            self.top_node = topA
            pipeline_stats['adjacency_chosen'] = 'A'
            if debug:
                print(f"[ADAPTIVE-PIPELINE] Chosen adjacency A, trunk_cost={costA:.3f}")
        else:
            self.chosen_adjacency = adjB
            self.cpoints_final = cpointsB
            self.node_map_final = node_mapB
            self.G_neg_final = GnegB
            self.trunk_path = trunkB
            self.base_node = baseB
            self.top_node = topB
            pipeline_stats['adjacency_chosen'] = 'B'
            if debug:
                print(f"[ADAPTIVE-PIPELINE] Chosen adjacency B, trunk_cost={costB:.3f}")
        
        # Invert graph direction
        self.G_neg_final = self.invert_graph_direction(self.G_neg_final, self.cpoints_final)
        
        # ---------------------------------------------------------------
        # 6) Label and refine
        # ---------------------------------------------------------------
        if debug:
            print("[ADAPTIVE-PIPELINE] Labeling trunk and leaves...")
        
        self.label_main_stem_and_leaves(self.G_neg_final, self.trunk_path)
        self.label_branch_off_nodes(self.G_neg_final, min_degree=3, debug=debug)
        
        # Build aggregated map
        ag_map = [None] * len(self.cpoints_final)
        for (si, cj), nd in self.node_map_final.items():
            ag_map[nd] = (si, cj)
        self.aggregated_centroids_map = ag_map
        
        # Refine branch points
        if debug:
            print("[DEBUG] Skipping refinement methods to preserve branch-off nodes...")
        # TEMPORARILY DISABLED: These methods are too aggressive and remove all branch-off nodes
        # self.refine_branch_off_nodes_immediate(self.G_neg_final, min_stem=2, min_leaf=1, debug=debug)
        # boffs = [nd for nd in self.G_neg_final.nodes()
        #         if self.G_neg_final.nodes[nd].get('type') == 'branch_off']
        # if debug:
        #     print(f"[DEBUG] Before refinement: {len(boffs)} branch-off nodes")
        # self.refine_close_branch_off_nodes(
        #     self.G_neg_final, self.trunk_path, boffs,
        #     trunk_distance_threshold=4, leaf_overlap_threshold=0.5
        # )
        self.branch_off_nodes = [nd for nd in self.G_neg_final.nodes()
                               if self.G_neg_final.nodes[nd].get('type') == 'branch_off']
        if debug:
            print(f"[DEBUG] After refinement: {len(self.branch_off_nodes)} branch-off nodes remain")
        
        # ---------------------------------------------------------------
        # 7) Create labeled point cloud
        # ---------------------------------------------------------------
        if debug:
            print("[ADAPTIVE-PIPELINE] Creating labeled point cloud...")
        
        self.labeled_pcd, labeled_arr = self.map_labels_to_original_points_unified(
            self.G_neg_final, self.slice_results, self.aggregated_centroids_map,
            self.original_pcd, output_path=output_path, base=base
        )
        
        if debug:
            print(f"[ADAPTIVE-PIPELINE] Pipeline completed successfully!")
            print(f"  - Created labeled PCD with {len(self.labeled_pcd.points)} points")
            print(f"  - Found {len(self.branch_off_nodes)} branch-off nodes")
            print(f"  - Trunk path has {len(self.trunk_path)} nodes")
            
            # Print performance comparison if available
            if 'method_A' in pipeline_stats.get('bridging_stats', {}):
                statsA = pipeline_stats['bridging_stats']['method_A']
                print(f"  - Method A bridging: {statsA['successful_bridges']} successful, "
                      f"{statsA['rejected_angle']} angle rejected")
            if 'method_B' in pipeline_stats.get('bridging_stats', {}):
                statsB = pipeline_stats['bridging_stats']['method_B']
                print(f"  - Method B bridging: {statsB['successful_bridges']} successful, "
                      f"{statsB['rejected_angle']} angle rejected")
        
        return self.branch_data, pipeline_stats



    ##########################################################################
    # Optional Visualization Helpers
    ##########################################################################

    def debug_print_graph(self, G=None):
        """
        Print node and edge information for debugging.

        Parameters
        ----------
        G : nx.Graph or nx.DiGraph, optional
            If None, uses self.G_neg_final.
        """
        if G is None:
            G= self.G_neg_final
        print("[DEBUG PRINT GRAPH]")
        if not G:
            print("No graph to print.")
            return
        print(f"  #nodes= {G.number_of_nodes()}, #edges= {G.number_of_edges()}")
        for nd in G.nodes():
            t= G.nodes[nd].get('type','?')
            coord= None
            if self.cpoints_final is not None and nd<len(self.cpoints_final):
                coord= self.cpoints_final[nd]
            print(f" Node={nd}, type={t}, coord= {coord}")
        print("Edges =>")
        for (u,v) in G.edges():
            w= G[u][v].get('weight', None)
            print(f"  Edge=({u}->{v}), w={w:.3f}" if w else f"  Edge=({u}->{v})")
        print("===")

    def visualize_final_graph_types(self):
        """
        Show the final G_neg_final with color-coded node types:
          'stem' -> red, 'branch_off' -> blue, 'leaf' -> green, etc.
        """
        if self.G_neg_final is None:
            print("[WARN] no final graph => run pipeline first.")
            return

        node_positions= []
        node_colors   = []
        nlist= list(self.G_neg_final.nodes())
        for nd in nlist:
            if (self.cpoints_final is not None) and (0<= nd < len(self.cpoints_final)):
                node_positions.append(self.cpoints_final[nd])
            else:
                node_positions.append([0,0,0])
            node_type= self.G_neg_final.nodes[nd].get('type','unknown')
            if node_type=='stem':
                node_colors.append([1,0,0])
            elif node_type=='branch_off':
                node_colors.append([0,0,1])
            elif node_type=='leaf':
                node_colors.append([0,1,0])
            else:
                node_colors.append([0.5,0.5,0.5])

        arr_pos= np.array(node_positions, dtype=float)
        arr_col= np.array(node_colors, dtype=float)

        pcd_nodes= o3d.geometry.PointCloud()
        pcd_nodes.points= o3d.utility.Vector3dVector(arr_pos)
        pcd_nodes.colors= o3d.utility.Vector3dVector(arr_col)

        edges=[]
        for nd in nlist:
            for nb in self.G_neg_final.neighbors(nd):
                if nb> nd:
                    edges.append([nd, nb])
        ls= o3d.geometry.LineSet()
        ls.points= o3d.utility.Vector3dVector(arr_pos)
        ls.lines = o3d.utility.Vector2iVector(np.array(edges,dtype=int))
        black= [[0,0,0]]* len(edges)
        ls.colors= o3d.utility.Vector3dVector(black)

        o3d.visualization.draw_geometries([pcd_nodes, ls], window_name="Final Graph + Types")

    def visualize_labeled_pcd(self):
        """
        Show the final color-coded point cloud after labeling (stem vs leaf).
        """
        if self.labeled_pcd is None:
            print("[WARN] No labeled_pcd => run_full_pipeline first.")
            return
        o3d.visualization.draw_geometries([self.labeled_pcd], window_name="Stem vs Leaf Labeled PC")