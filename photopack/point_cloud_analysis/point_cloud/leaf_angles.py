import numpy as np
from collections import deque
import open3d as o3d
from sklearn.linear_model import LinearRegression


class LeafAngleAnalyzer:
    """
    LeafAngleAnalyzer computes leaf angles from segmented plant point cloud data.

    It expects that the segmentation instance (from MainStemSegmentation) 
    has already identified branch 'stem' vs. 'leaf' points for each branch, 
    typically stored as:
      - all_main_stem_points_up[i]: Nx3 array (the 'up' portion of the main stem)
      - all_main_stem_points_down[i]: Nx3 array (the 'down' portion of the main stem)
      - all_leaf_points[i]: Nx3 array of leaf region points

    This class can then:
      - Compute leaf angles by performing a linear regression on (z vs. x,y) 
        for the main stem and the leaf.
      - Visualize these angles in Open3D by drawing arcs and lines.

    Attributes
    ----------
    segmentation : MainStemSegmentation (or similar)
        An instance that holds references to 'all_main_stem_points_up', 
        'all_main_stem_points_down', and 'all_leaf_points'.
    angles : list of float or None
        Computed leaf angles (in degrees) for each branch. If a branch 
        does not meet the minimum data requirement, its angle is None.
    main_leaf_angle : float or None
        An optional metric for the "main leaf angle" (e.g., topmost leaf).
    main_leaf_branch_index : int or None
        Which branch index had the "main leaf angle".
    """

    def __init__(self, segmentation):
        """
        Initialize with a segmentation object that has 
        main-stem and leaf points for each branch region.

        Parameters
        ----------
        segmentation : object
            Should contain:
              segmentation.all_main_stem_points_up
              segmentation.all_main_stem_points_down
              segmentation.all_leaf_points
            each a list of Nx3 arrays.
        """
        self.segmentation = segmentation
        self.angles = []  # Computed leaf angles per branch
        self.main_leaf_angle = None
        self.main_leaf_branch_index = None



    def compute_leaf_angles_node_bfs(
        self,
        n_main_stem=5,
        n_leaf=5,
        flip_if_obtuse=True,
        min_leaf_for_angle=4,
        max_bfs_depth=5
    ):
        """
        Replicate the BFS approach from your segmentation code:
         - For each branch_off node in the trunk, gather 'stem_points' around it
         - Collect connected 'leaf_points' by BFS
         - Fit lines (SVD in 3D) and compute the angle

        Parameters
        ----------
        n_main_stem : int
            How many trunk nodes above/below the branch_off node we gather.
        n_leaf : int
            Max leaf nodes to collect by BFS.
        flip_if_obtuse : bool
            If True, angles >90 deg => use (180 - angle).
        min_leaf_for_angle : int
            If BFS finds fewer leaf nodes => skip angle.
        max_bfs_depth : int
            BFS limit for counting leaves if you want to ensure enough leaf points.

        Returns
        -------
        self.branch_data : list of dict
            Each dict = {
               'branch_off'  : node_id,
               'stem_points' : Nx3,
               'leaf_points' : Mx3,
               'angle_degrees' : float
            }
        """
        G         = self.segmentation.G_neg_final
        cpoints   = self.segmentation.cpoints_final
        trunk_ids = self.segmentation.trunk_path
        b_offs    = self.segmentation.branch_off_nodes

        if G is None or cpoints is None or not trunk_ids:
            print("[ERROR] Segmentation data missing or invalid.")
            return []

        # We'll define some BFS helpers
        def undirected_neighbors(u):
            return set(G.successors(u)) | set(G.predecessors(u))

        def count_leaf_nodes_bfs(start, depth_limit):
            visited = set()
            queue   = deque([(start,0)])
            leaf_cnt= 0
            while queue:
                nd, dpt = queue.popleft()
                if nd in visited:
                    continue
                visited.add(nd)
                if G.nodes[nd].get('type')=='leaf':
                    leaf_cnt += 1
                if dpt< depth_limit:
                    for nb in undirected_neighbors(nd):
                        if nb not in visited:
                            queue.append((nb, dpt+1))
            return leaf_cnt

        def collect_leaf_nodes_bfs(start, n_leaf):
            visited = set()
            queue   = deque()
            leaves  = []
            # Start from immediate neighbors that are 'leaf'
            for nb in undirected_neighbors(start):
                if G.nodes[nb].get('type')=='leaf':
                    queue.append(nb)

            while queue and len(leaves)< n_leaf:
                curr = queue.popleft()
                if curr in visited:
                    continue
                visited.add(curr)
                if G.nodes[curr].get('type')=='leaf':
                    leaves.append(curr)
                    # expand further leaf neighbors
                    for nxt in undirected_neighbors(curr):
                        if nxt not in visited and G.nodes[nxt].get('type')=='leaf':
                            queue.append(nxt)
            return leaves

        def fit_line_svd(pts):
            """
            SVD-based line fit in 3D. Returns (center, direction).
            """
            arr = np.asarray(pts)
            if arr.shape[0]<2:
                return (None,None)
            center = arr.mean(axis=0)
            uu, ss, vh = np.linalg.svd(arr - center)
            direction  = vh[0] / np.linalg.norm(vh[0])
            return (center, direction)

        self.branch_data = []
        trunk_set = set(trunk_ids)

        for b_off in b_offs:
            # Ensure the branch_off node is actually on trunk
            if b_off not in trunk_set:
                print(f"[WARN] skip b_off= {b_off}, not on trunk_path.")
                continue

            # Count leaf nodes within BFS => skip if not enough
            leaf_count = count_leaf_nodes_bfs(b_off, max_bfs_depth)
            if leaf_count < min_leaf_for_angle:
                print(f"[SKIP] branch_off= {b_off}: only {leaf_count} leaves => skip angle.")
                continue

            # Collect trunk nodes: n_main_stem above + below
            idx      = trunk_ids.index(b_off)
            aboveIDs = trunk_ids[idx+1 : idx+1+n_main_stem]
            belowIDs = trunk_ids[max(0, idx-n_main_stem) : idx]
            trunk_nodes  = aboveIDs + belowIDs
            trunk_points = []
            for tn in trunk_nodes:
                if tn < len(cpoints):
                    trunk_points.append(cpoints[tn])
            # if we don't have at least 2 points => skip
            if len(trunk_points) < 2:
                continue

            # Collect leaf nodes => BFS
            leaf_nodes = collect_leaf_nodes_bfs(b_off, n_leaf)
            leaf_points= []
            for ln in leaf_nodes:
                if ln < len(cpoints):
                    leaf_points.append(cpoints[ln])
            if len(leaf_points) < 2:
                continue

            # Fit lines
            stem_center, stem_dir = fit_line_svd(trunk_points)
            leaf_center, leaf_dir = fit_line_svd(leaf_points)
            if stem_dir is None or leaf_dir is None:
                continue

            # Compute angle
            dot_val = np.clip(np.dot(stem_dir, leaf_dir), -1.0, 1.0)
            angle_deg = np.degrees(np.arccos(dot_val))
            if flip_if_obtuse and angle_deg>90:
                angle_deg = 180 - angle_deg

            self.branch_data.append({
                'branch_off':   b_off,
                'stem_points':  np.array(trunk_points),
                'leaf_points':  np.array(leaf_points),
                'angle_degrees': angle_deg
            })
            print(f"[NODE-BFS ANGLE] b_off= {b_off}, angle= {angle_deg:.2f}")

        # Pull out angles alone
        self.angles = [bd['angle_degrees'] for bd in self.branch_data]

        # Optionally pick a "main_leaf_angle" if you want the largest or something
        best_idx, best_val = None, None
        for i, bd in enumerate(self.branch_data):
            ang = bd['angle_degrees']
            if best_val is None or (ang is not None and ang> best_val):
                best_val = ang
                best_idx= i
        self.main_leaf_angle = best_val
        self.main_leaf_branch_index = best_idx

        return self.branch_data


    def visualize_bfs_leaf_angles_open3d(self):
        """
        Example visualization that draws:
         - trunk_points in red
         - leaf_points in green
         - a line for trunk, a line for leaf
         - an arc representing the angle

        BFS-based approach: we pull data from self.branch_data, where
        trunk_points & leaf_points were found by BFS.
        """

        import open3d as o3d

        geoms = []
        for i, bd in enumerate(self.branch_data):
            trunk_pts = bd['stem_points']
            leaf_pts  = bd['leaf_points']
            angle_deg = bd['angle_degrees']

            # Build small point clouds
            trunk_pcd = o3d.geometry.PointCloud()
            trunk_pcd.points = o3d.utility.Vector3dVector(trunk_pts)
            trunk_pcd.paint_uniform_color([1,0,0])  # red
            geoms.append(trunk_pcd)

            leaf_pcd  = o3d.geometry.PointCloud()
            leaf_pcd.points = o3d.utility.Vector3dVector(leaf_pts)
            leaf_pcd.paint_uniform_color([0,1,0])  # green
            geoms.append(leaf_pcd)


            print(f"Branch {i} => angle= {angle_deg:.2f} deg")

        # show them
        if geoms:
            o3d.visualization.draw_geometries(geoms, window_name="BFS Leaf Angles")
        else:
            print("[WARN] No BFS angles to visualize or empty branch_data.")



    # --------------------------------------------------------------------
    # Visualization of Leaf Angles in Open3D
    # --------------------------------------------------------------------

    def visualize_leaf_angles(self):
        """
        Display in Open3D each branch’s trunk/leaf lines plus an arc
        representing the BFS-based angle between them. Adds a small sphere at
        the midpoint of the arc for better visual reference.
        """
        geoms = []
        for i, bd in enumerate(self.branch_data):
            trunk_pts = bd['stem_points']
            leaf_pts  = bd['leaf_points']
            angle_deg = bd.get('angle_degrees', None)

            if trunk_pts.shape[0] < 2 or leaf_pts.shape[0] < 2:
                print(f"[WARN] branch {i} => not enough trunk/leaf points => skipping.")
                continue

            # (A) point clouds for raw BFS-based data
            geoms += self._pcd_from_points(trunk_pts, color=[1,0,0])  # red for stem
            geoms += self._pcd_from_points(leaf_pts,  color=[0,1,0])  # green for leaf

            # (B) Fit lines in 3D using SVD => sample Nx3 line
            trunk_reg_pts = self._fit_line_svd_as_points(trunk_pts, n_samples=30)
            leaf_reg_pts  = self._fit_line_svd_as_points(leaf_pts,  n_samples=30)

            # (C) create line sets for trunk & leaf "regression lines"
            geoms += self._lineset_from_points(trunk_reg_pts, color=[0,0,1])   # blue line
            geoms += self._lineset_from_points(leaf_reg_pts,  color=[1,0,1])   # magenta line

            # (D) add the arc + sphere at midpoint
            if trunk_reg_pts.shape[0]>=2 and leaf_reg_pts.shape[0]>=2:
                arc_geoms = self._arc_for_angle(
                    trunk_reg_pts[0],
                    trunk_reg_pts[-1],
                    leaf_reg_pts[-1]
                )
                geoms.extend(arc_geoms)

            # Finally, print the BFS-based angle
            if angle_deg is not None:
                print(f"[INFO] branch {i} => BFS-based angle= {angle_deg:.2f} deg")
            else:
                print(f"[INFO] branch {i} => BFS-based angle= ??? (not in branch_data)")

        if not geoms:
            print("[WARN] no geometry => no BFS angles to display.")
            return

        # Display in Open3D
        o3d.visualization.draw_geometries(geoms, window_name="BFS Leaf Angles w/ Arc Spheres")

    # --------------------------------------------------------------------
    # Helpers for geometry creation and line fitting
    # --------------------------------------------------------------------

    def _pcd_from_points(self, points, color=[1,0,0]):
        """
        Helper: create a small geometry list with a single PointCloud of 'points'.
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color(color)
        return [pcd]

    def _lineset_from_points(self, pts, color=[0,0,1]):
        """
        Given Nx3 points (like a 'regression line'), build a line set in a single color.
        """
        if len(pts) < 2:
            return []
        lines = [[i, i+1] for i in range(len(pts)-1)]
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(pts)
        ls.lines  = o3d.utility.Vector2iVector(lines)
        ls.colors = o3d.utility.Vector3dVector([color]*len(lines))
        return [ls]

    def _fit_line_svd_as_points(self, pts, n_samples=30):
        """
        Similar to 'regression' but in 3D with SVD:
          - Fit principal axis for 'pts'
          - Create line from min_proj..max_proj at intervals => Nx3 array

        Returns Nx3 array of sampled points along the best-fit line.
        """
        arr = np.asarray(pts)
        center = arr.mean(axis=0)
        if arr.shape[0] < 2:
            return arr  # degenerate

        # SVD => first principal component
        uu, ss, vh = np.linalg.svd(arr - center)
        direction  = vh[0] / np.linalg.norm(vh[0])  # principal axis

        # Project all points onto that axis => find min/max
        projs = np.dot((arr - center), direction)  # shape (N,)
        min_t, max_t = np.min(projs), np.max(projs)

        # Sample n_samples points from min_t..max_t
        ts = np.linspace(min_t, max_t, n_samples)
        line_pts = [center + t*direction for t in ts]
        return np.array(line_pts)

    # --------------------------------------------------------------------
    # Arc creation + a sphere at its midpoint
    # --------------------------------------------------------------------

    def _arc_for_angle(self, stem_start, stem_end, leaf_end):
        """
        Creates:
          1) A LineSet for the arc from (stem_start->stem_end) to (stem_start->leaf_end)
          2) A small sphere at the midpoint of that arc

        Returns
        -------
        geoms : list of open3d.geometry.Geometry
            [arc_lineset, sphere_at_mid]
        """
        arc_pts = self._create_arc_points(stem_start, stem_end, leaf_end, num_points=30)
        if arc_pts.shape[0] < 2:
            return []  # no arc

        # (1) build a lineset for the arc
        arc_lines = [[i, i+1] for i in range(len(arc_pts)-1)]
        arc_ls = o3d.geometry.LineSet()
        arc_ls.points = o3d.utility.Vector3dVector(arc_pts)
        arc_ls.lines  = o3d.utility.Vector2iVector(arc_lines)
        arc_ls.colors = o3d.utility.Vector3dVector([[0,0,0]]* len(arc_lines))  # yellow

        # (2) small sphere at midpoint
        mid_idx = len(arc_pts)//2
        mid_pt  = arc_pts[mid_idx]
        sphere  = self._create_sphere_at_point(mid_pt, radius=0.002, color=[0,0,0])

        return [arc_ls, sphere]

    def _create_arc_points(self, center, end1, end2, num_points=30):
        """
        Create a set of 3D arc points from the vectors (center->end1) to (center->end2).
        """
        v1 = end1 - center
        v2 = end2 - center
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1<1e-9 or norm2<1e-9:
            return np.vstack([center, end1, end2])  # degenerate

        v1u = v1 / norm1
        v2u = v2 / norm2
        dotv= np.clip(v1u.dot(v2u), -1,1)
        angle= np.arccos(dotv)
        if angle< 1e-9:
            return np.vstack([center, end1])  # minimal angle => no arc
        axis = np.cross(v1u, v2u)
        a_len= np.linalg.norm(axis)
        if a_len<1e-12:
            # collinear
            return np.vstack([center, end1, end2])
        axis /= a_len

        arc_pts= []
        for i in range(num_points+1):
            t= angle*(i/num_points)
            rot_v = self._rotate_vector_around_axis(v1, axis, t)
            arc_pts.append(center+ rot_v)
        return np.array(arc_pts)

    def _rotate_vector_around_axis(self, vec, axis, theta):
        """
        Rodrigues rotation formula: v_rot = v*cosθ + (k×v)*sinθ + k*(k·v)*(1−cosθ).
        """
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        k = axis
        v = vec
        v_cos = v * cos_t
        k_cross_v = np.cross(k, v)
        v_sin = k_cross_v * sin_t
        k_dot_v = np.dot(k, v)
        v_k = k * (k_dot_v*(1 - cos_t))
        return v_cos + v_sin + v_k

    def _create_sphere_at_point(self, center, radius=0.00005, color=[0,0,0]):
        """
        Create an Open3D sphere geometry placed at 'center' with the given 'radius'.
        """
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.compute_vertex_normals()
        # Move the sphere so its center is at 'center'
        sphere.translate(center)
        # Color the entire mesh
        sphere.paint_uniform_color(color)
        return sphere

