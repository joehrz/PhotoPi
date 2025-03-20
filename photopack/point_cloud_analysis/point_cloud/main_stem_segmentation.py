import copy
import math
import numpy as np
import open3d as o3d
import networkx as nx
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
from sklearn.linear_model import LinearRegression
from scipy.optimize import linear_sum_assignment
from collections import deque
import matplotlib.pyplot as plt
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




    # def compute_robust_principal_axis(self):
    #     """
    #     Perform a robust PCA to find trunk axis by focusing on upper portion of plant.
    
    #     Returns
    #     -------
    #     axis : np.ndarray (3,)
    #         The principal axis of the centroids, oriented so that axis[2]>=0.
    #     """
    #     all_c = []
    #     z_values = []
    
    #     # Collect all centroids and their z-values
    #     for slc in self.slice_results:
    #         for cd in slc:
    #             all_c.append(cd['centroid'])
    #             z_values.append(cd['centroid'][2])
    
    #     if len(all_c) < 2:
    #         return np.array([0, 0, 1], dtype=float)
    
    #     arr = np.array(all_c)
    #     z_values = np.array(z_values)
    
    #     # Find z-height cutoff (use upper half of plant)
    #     z_cutoff = np.median(z_values)
    
    #     # Use only points in upper portion for initial PCA
    #     upper_mask = z_values > z_cutoff
    #     if np.sum(upper_mask) >= 3:  # Need minimum points for PCA
    #         upper_arr = arr[upper_mask]
    #         upper_centered = upper_arr - upper_arr.mean(axis=0)
    #         _, _, vT = np.linalg.svd(upper_centered, full_matrices=False)
    #         axis = vT[0]
    #         if axis[2] < 0:
    #             axis = -axis
    #     else:
    #         # Fallback to vertical if not enough points
    #         axis = np.array([0, 0, 1], dtype=float)
    
    #     return axis











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










    # def build_raindrop_negcost_digraph(self, cpoints, adjacency, node_map,
    #                                    alpha=1.0, beta=1.0, trunk_axis=None,
    #                                    reverse_z=False, debug=True):
    #     """
    #     Build a negative-cost DAG for trunk extraction:
    #      - If not reverse_z: edge (u->v) if z_v < z_u, cost= -(alpha*horiz + beta*angle).
    #      - If reverse_z:     edge (v->u) if z_v >= z_u, symmetrical logic.

    #     Parameters
    #     ----------
    #     cpoints : np.ndarray
    #         (N,3) centroids.
    #     adjacency : dict
    #         The adjacency dict from bipartite matching.
    #     node_map : dict
    #         (slice_i, cluster_j) -> node_id
    #     alpha : float
    #         Factor for horizontal cost.
    #     beta : float
    #         Factor for angle cost.
    #     trunk_axis : np.ndarray or None
    #         The approximate vertical axis; if None, use [0,0,-1].
    #     reverse_z : bool
    #         If True, we invert the direction logic: 
    #         edges go from higher-z to lower-z.
    #     debug : bool
    #         If True, prints extra debug info.

    #     Returns
    #     -------
    #     G : nx.DiGraph
    #         The negative-cost DAG for trunk extraction.
    #     """
    #     G = nx.DiGraph()
    #     N = len(cpoints)
    #     for nd in range(N):
    #         G.add_node(nd)

    #     if trunk_axis is None:
    #         ref_axis = np.array([0,0,-1], dtype=float)
    #     else:
    #         ref_axis = trunk_axis / (np.linalg.norm(trunk_axis)+1e-12)
    #         if ref_axis[2]>0:
    #             ref_axis = -ref_axis

    #     def measure_angle(vec, axis):
    #         mag = np.linalg.norm(vec)
    #         if mag<1e-12:
    #             return 0.0
    #         dotv = max(-1.0, min(1.0, vec.dot(axis)/mag))
    #         return math.acos(dotv)

    #     total_edges= 0
    #     for key, neighbors in adjacency.items():
    #         ndA= node_map[key]
    #         zA = cpoints[ndA][2]
    #         for nb in neighbors:
    #             ndB= node_map[nb]
    #             zB = cpoints[ndB][2]

    #             if not reverse_z:
    #                 # normal => edge if zB < zA
    #                 if zB< zA:
    #                     vec = cpoints[ndB] - cpoints[ndA]
    #                     horiz= np.linalg.norm(vec[:2])
    #                     ang  = measure_angle(vec, ref_axis)
    #                     cost_uv= -(alpha*horiz + beta*ang)
    #                     G.add_edge(ndA, ndB, weight= cost_uv)
    #                     total_edges+=1
    #             else:
    #                 # reversed => edge if zB >= zA => (ndB->ndA)
    #                 if zB>= zA:
    #                     vec= cpoints[ndA] - cpoints[ndB]
    #                     horiz= np.linalg.norm(vec[:2])
    #                     ang  = measure_angle(vec, ref_axis)
    #                     cost_vu= -(alpha*horiz + beta*ang)
    #                     G.add_edge(ndB, ndA, weight= cost_vu)
    #                     total_edges+=1

    #     if debug:
    #         print(f"[DEBUG] build_raindrop_negcost_digraph => #edges= {total_edges}")
    #     return G

    # def longest_path_in_DAG_negcost(self, G, top_node_id, debug=True):
    #     """
    #     Given a negative-cost DAG, find the path that yields minimal sum 
    #     (which is effectively the 'longest' path in positive sense).

    #     We do a topological sort, then DP to pick min-sum path from top_node_id.

    #     Parameters
    #     ----------
    #     G : nx.DiGraph
    #         The negative-cost DAG.
    #     top_node_id : int
    #         The node from which we begin the path (highest z).
    #     debug : bool
    #         If True, prints debug info.

    #     Returns
    #     -------
    #     base_node : int
    #         The node at the other end of the minimal-cost path.
    #     path : list of int
    #         The sequence of nodes in that trunk path.
    #     """
    #     topo_order= list(nx.topological_sort(G))
    #     if top_node_id not in topo_order:
    #         # fallback
    #         return (top_node_id, [top_node_id])

    #     maxDist= {}
    #     pred   = {}
    #     for nd in G.nodes():
    #         maxDist[nd]= float('inf')  # we want minimal sum => store inf
    #         pred[nd]= None
    #     maxDist[top_node_id]= 0.0

    #     for nd in topo_order:
    #         if maxDist[nd]== float('inf'):
    #             continue
    #         cost_nd= maxDist[nd]
    #         for nb in G[nd]:
    #             w= G[nd][nb]['weight']
    #             alt= cost_nd + w
    #             if alt< maxDist[nb]:
    #                 maxDist[nb]= alt
    #                 pred[nb]= nd

    #     best_node= top_node_id
    #     best_val = maxDist[top_node_id]
    #     for nd in G.nodes():
    #         if maxDist[nd]< best_val:
    #             best_val= maxDist[nd]
    #             best_node= nd
    #     path=[]
    #     tmp= best_node
    #     while tmp is not None:
    #         path.append(tmp)
    #         tmp= pred[tmp]
    #     path.reverse()
    #     return (best_node, path)

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

    # def label_branch_off_nodes(self, G, min_degree=3):
    #     """
    #     Mark any node with degree >= min_degree as 'branch_off'.

    #     Parameters
    #     ----------
    #     G : nx.Graph or nx.DiGraph
    #         The graph in which to label nodes.
    #     min_degree : int
    #         Degree threshold for labeling branch_off.
    #     """
    #     for nd in G.nodes():
    #         if G.degree(nd) >= min_degree:
    #             G.nodes[nd]['type'] = 'branch_off'

    def label_branch_off_nodes(self, G, min_degree=3):
        """
        Mark any node with degree >= min_degree as 'branch_off'.
        
        Parameters
        ----------
        G : nx.Graph or nx.DiGraph
            The graph in which to label nodes.
        min_degree : int
            Degree threshold for labeling branch_off.
        """
        branch_points = []
        for nd in G.nodes():
            if G.degree(nd) >= min_degree:
                G.nodes[nd]['type'] = 'branch_off'
                G.nodes[nd]['branch_count'] = G.degree(nd) - 2  # Number of potential branches
                branch_points.append(nd)
        return branch_points

    def refine_branch_off_nodes_immediate(self, G, min_stem=2, min_leaf=1):
        """
        For each 'branch_off', check immediate neighbors. 
        If fewer than (min_stem) stem neighbors or (min_leaf) leaf neighbors, 
        re-label based on majority. 
        """
        def undirected_neighbors(u):
            return set(G.successors(u)) | set(G.predecessors(u))

        new_labels = {}
        for nd in G.nodes():
            if G.nodes[nd].get('type')=='branch_off':
                nbrs= undirected_neighbors(nd)
                stem_count= sum(G.nodes[x].get('type')=='stem' for x in nbrs)
                leaf_count= sum(G.nodes[x].get('type')=='leaf' for x in nbrs)
                if stem_count< min_stem or leaf_count< min_leaf:
                    if stem_count> leaf_count:
                        new_labels[nd]= 'stem'
                    elif leaf_count> stem_count:
                        new_labels[nd]= 'leaf'
                    else:
                        new_labels[nd]= 'outlier'
        for k,v in new_labels.items():
            G.nodes[k]['type']= v

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

    def map_labels_to_original_points_unified(self, G, slice_results, aggregated_map):
        """
        Map each graph node type back to the original points. 
        Branch_off => unify as 'stem'.

        Parameters
        ----------
        G : nx.Graph or nx.DiGraph
            The final labeled graph.
        slice_results : list of list of dict
            The same slice cluster data structure.
        aggregated_map : list of tuple
            Indexed by node_id => (slice_i, cluster_j).

        Returns
        -------
        out_pcd : o3d.geometry.PointCloud
            Color-coded by 'stem'(red), 'leaf'(green), 'unknown'(gray).
        labeled_arr : np.ndarray, shape (N,4)
            (x, y, z, label_int), where label_int=0(stem),1(leaf),2(unknown).
        """
        color_map= {
            'stem':    [1,0,0],
            'leaf':    [0,1,0],
            'unknown': [0.6,0.6,0.6]
        }
        label_map= {
            'stem':0,
            'leaf':1,
            'unknown':2
        }

        labeled_pts= []
        labeled_cols= []
        labeled_arr= []

        for nd in G.nodes():
            t= G.nodes[nd].get('type','unknown')
            if t=='branch_off':
                t='stem'
            col= color_map.get(t, [0.6,0.6,0.6])
            lbl= label_map.get(t,2)

            si, cj= aggregated_map[nd]
            pts= slice_results[si][cj]['points']
            for p in pts:
                labeled_pts.append(p)
                labeled_cols.append(col)
                labeled_arr.append([p[0], p[1], p[2], lbl])

        out_pcd= o3d.geometry.PointCloud()
        arr_pts= np.array(labeled_pts, dtype=float)
        arr_col= np.array(labeled_cols, dtype=float)
        out_pcd.points= o3d.utility.Vector3dVector(arr_pts)
        out_pcd.colors= o3d.utility.Vector3dVector(arr_col)

        labeled_arr= np.array(labeled_arr, dtype=float)
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
    ):
        """
        Reorder the operations to match the standalone script’s logic/sequence:
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
                min_samples=25, 
                dist_merge=0.02, 
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
        self.label_branch_off_nodes(self.G_neg_final, min_degree=3)

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
            self.aggregated_centroids_map
        )
        if debug:
            print(f"[INFO] Created labeled PCD => #points= {len(self.labeled_pcd.points)}")
            print("[END OF PIPELINE] => main stem & branch_off labeled => done.")

        return self.branch_data



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


