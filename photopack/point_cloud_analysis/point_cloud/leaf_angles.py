import numpy as np
from collections import deque
import open3d as o3d




class LeafAngleAnalyzer:
    """
    Compute precise stem-to-leaf divergence angles from an attributed
    point-cloud segmentation.

    The constructor expects *segmentation* to provide **four** public members:

    • ``G_neg_final``        - directed ``networkx.DiGraph`` of cluster nodes  
    • ``cpoints_final``      - ``(N, 3)`` centroid coordinates (x, y, z)  
    • ``trunk_path``         - ordered node IDs of the main stem (base→apex)  
    • ``branch_off_nodes``   - node IDs where lateral branches emerge  

    Workflow
    --------
    1. Breadth-first search samples stem and leaf nodes around each
       ``branch_off`` point.  
    2. Singular-value decomposition fits a straight line to each sample.  
    3. The angle between stem and leaf lines is reported in raw, acute,
       obtuse, and final forms.  
    4. The branch at the greatest *z* height is flagged as the **main leaf**.

    Key Attributes
    --------------
    branch_data : List[dict]  
        One record per analysed branch with fields:

        ``branch_off``   - origin node ID  
        ``branch_z``     - height (z) of the branch-off node  
        ``stem_points``  - sampled stem points  
        ``leaf_points``  - sampled leaf points  
        ``angle_raw``    - 0 - 180°  
        ``angle_acute``  - ≤ 90°  
        ``angle_obtuse`` - 180° -acute  
        ``angle_final``  - selected by *angle_mode*  

    angles : List[float]           - shorthand list of ``angle_final`` values  
    main_leaf_angle : float | None - angle of the highest branch (if any)  
    main_leaf_branch_index : int | None - index in ``branch_data`` for the
        main leaf

    Principal Methods
    -----------------
    compute_leaf_angles_node_bfs(...)  
        Runs the full analysis and populates ``branch_data``.  

    visualize_samples_open3d()  
        Shows only the sampled stem (red) and leaf (green) points.  

    visualize_leaf_angles(arc_samples=30)  
        Renders fitted lines plus an arc illustrating each measured angle.

    Example
    -------
    >>> analyzer = LeafAngleAnalyzer(segmentation)
    >>> analyzer.compute_leaf_angles_node_bfs(angle_mode="auto")
    >>> print(f"Primary leaf angle: {analyzer.main_leaf_angle:.1f}°")
    >>> analyzer.visualize_leaf_angles()
    """
    # ------------------------------------------------------------------ #
    # Construction                                                       #
    # ------------------------------------------------------------------ #

    def __init__(self, segmentation):
        """
        segmentation  must expose
            • G_neg_final          - directed NetworkX graph
            • cpoints_final        - (N, 3) centroid coords
            • trunk_path           - list[int] main-stem node IDs
            • branch_off_nodes     - list[int] branch-off node IDs
        """
        self.segmentation = segmentation
        self.branch_data = []
        self.angles = []
        self.main_leaf_angle = None
        self.main_leaf_branch_index = None

    # ------------------------------------------------------------------ #
    # Core computation                                                   #
    # ------------------------------------------------------------------ #

    def compute_leaf_angles_node_bfs(
        self,
        n_main_stem=5,
        n_leaf=5,
        min_leaf_for_angle=4,
        max_bfs_depth=5,
        angle_mode="auto"
    ):
        """Compute branch angles and mark the highest-z one as the main leaf."""

        if angle_mode not in {"acute", "obtuse", "raw", "auto"}:
            raise ValueError("angle_mode must be 'acute', 'obtuse', 'raw', or 'auto'")

        G = self.segmentation.G_neg_final
        cpoints = self.segmentation.cpoints_final
        trunk_ids = self.segmentation.trunk_path
        b_offs = self.segmentation.branch_off_nodes

        if G is None or cpoints is None or not trunk_ids:
            print("[ERROR] Segmentation data missing or invalid.")
            return []

        # ---------- helpers -------------------------------------------------
        def undirected_neighbors(u):
            return set(G.successors(u)) | set(G.predecessors(u))

        def count_leaf_nodes_bfs(start, depth_limit):
            visited, q = set(), deque([(start, 0)])
            cnt = 0
            while q:
                nd, depth = q.popleft()
                if nd in visited:
                    continue
                visited.add(nd)
                if G.nodes[nd].get("type") == "leaf":
                    cnt += 1
                if depth < depth_limit:
                    for nb in undirected_neighbors(nd):
                        if nb not in visited:
                            q.append((nb, depth + 1))
            return cnt

        def collect_leaf_nodes_bfs(start, n_max):
            visited, q, leaves = set(), deque(), []
            for nb in undirected_neighbors(start):
                if G.nodes[nb].get("type") == "leaf":
                    q.append(nb)
            while q and len(leaves) < n_max:
                nd = q.popleft()
                if nd in visited:
                    continue
                visited.add(nd)
                if G.nodes[nd].get("type") == "leaf":
                    leaves.append(nd)
                    for nb in undirected_neighbors(nd):
                        if nb not in visited and G.nodes[nb].get("type") == "leaf":
                            q.append(nb)
            return leaves

        def fit_line_svd(pts):
            arr = np.asarray(pts)
            if arr.shape[0] < 2:
                return None, None
            center = arr.mean(axis=0)
            _, _, vh = np.linalg.svd(arr - center, full_matrices=False)
            direction = vh[0] / np.linalg.norm(vh[0])
            return center, direction

        # ---------- main loop ----------------------------------------------
        self.branch_data.clear()

        for b_off in b_offs:
            if count_leaf_nodes_bfs(b_off, max_bfs_depth) < min_leaf_for_angle:
                continue

            branch_z = cpoints[b_off][2]           

            # --- stem sample
            idx = trunk_ids.index(b_off) if b_off in trunk_ids else -1
            trunk_nodes = (
                trunk_ids[idx + 1 : idx + 1 + n_main_stem]
                + trunk_ids[max(0, idx - n_main_stem) : idx]
            )
            trunk_pts = [cpoints[tn] for tn in trunk_nodes if tn < len(cpoints)]
            if len(trunk_pts) < 2:
                continue

            # --- leaf sample
            leaf_nodes = collect_leaf_nodes_bfs(b_off, n_leaf)
            leaf_pts = [cpoints[ln] for ln in leaf_nodes if ln < len(cpoints)]
            if len(leaf_pts) < 2:
                continue

            stem_center, stem_dir = fit_line_svd(trunk_pts)
            leaf_center, leaf_dir = fit_line_svd(leaf_pts)
            if stem_dir is None or leaf_dir is None:
                continue

            # --- deterministic orientation
            if stem_dir[2] < 0:
                stem_dir *= -1
            if np.dot(leaf_dir, leaf_center - stem_center) < 0:
                leaf_dir *= -1

            # --- angle maths
            dot_val = float(np.clip(np.dot(stem_dir, leaf_dir), -1.0, 1.0))
            theta_raw = float(np.degrees(np.arccos(dot_val)))
            theta_acute = theta_raw if theta_raw <= 90 else 180 - theta_raw
            theta_obtuse = 180 - theta_acute

            if angle_mode == "acute":
                theta_final = theta_acute
            elif angle_mode == "obtuse":
                theta_final = theta_obtuse
            elif angle_mode == "raw":
                theta_final = theta_raw
            else:  # "auto"
                theta_final = theta_acute if leaf_dir[2] >= 0 else theta_obtuse

            self.branch_data.append(
                dict(
                    branch_off   = b_off,
                    branch_z     = branch_z,          
                    stem_points  = np.asarray(trunk_pts),
                    leaf_points  = np.asarray(leaf_pts),
                    angle_raw    = theta_raw,
                    angle_acute  = theta_acute,
                    angle_obtuse = theta_obtuse,
                    angle_final  = theta_final,
                )
            )

        # ------------- select MAIN leaf (highest z) ------------------------
        if self.branch_data:
            z_vals = [bd["branch_z"] for bd in self.branch_data]
            self.main_leaf_branch_index = int(np.argmax(z_vals))
            self.main_leaf_angle = self.branch_data[
                self.main_leaf_branch_index
            ]["angle_final"]
        else:
            self.main_leaf_branch_index = None
            self.main_leaf_angle = None

        self.angles = [bd["angle_final"] for bd in self.branch_data]
        return self.branch_data

    # ------------------------------------------------------------------ #
    # Quick visualisation of BFS samples                                 #
    # ------------------------------------------------------------------ #

    def visualize_samples_open3d(self):
        geoms = []
        for i, bd in enumerate(self.branch_data):
            geoms += self._pcd_from_points(bd["stem_points"], [1, 0, 0])
            geoms += self._pcd_from_points(bd["leaf_points"], [0, 1, 0])
            star = " ⭑" if i == self.main_leaf_branch_index else ""
            print(
                f"branch {i:2d}{star}  z={bd['branch_z']:.3f}  "
                f"angle={bd['angle_final']:.1f}°"
            )
        if geoms:
            o3d.visualization.draw_geometries(geoms)
        else:
            print("[WARN] No geometry to display.")

    # ------------------------------------------------------------------ #
    # Visualisation (lines + arc)                                 #
    # ------------------------------------------------------------------ #

    def visualize_leaf_angles(self, arc_samples=30):
        if not self.branch_data:
            print("[WARN] No branch data to display.")
            return

        geoms = []
        for i, bd in enumerate(self.branch_data):
            stem_pts, leaf_pts = bd["stem_points"], bd["leaf_points"]

            geoms += self._pcd_from_points(stem_pts, [1, 0, 0])
            geoms += self._pcd_from_points(leaf_pts, [0, 1, 0])

            stem_line = self._fit_line_svd_as_points(stem_pts, 30)
            leaf_line = self._fit_line_svd_as_points(leaf_pts, 30)

            geoms += self._lineset_from_points(stem_line, [0, 0, 1])
            geoms += self._lineset_from_points(leaf_line, [1, 0, 1])

            if stem_line.shape[0] >= 2 and leaf_line.shape[0] >= 2:
                geoms += self._arc_for_angle(
                    stem_line[0], stem_line[-1], leaf_line[-1], arc_samples
                )

        o3d.visualization.draw_geometries(geoms)

    # -------------- helper geometry builders ----------
    @staticmethod
    def _pcd_from_points(points, color):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.paint_uniform_color(color)
        return [pcd]

    @staticmethod
    def _lineset_from_points(pts, color):
        if len(pts) < 2:
            return []
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(pts)
        ls.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(pts) - 1)])
        ls.colors = o3d.utility.Vector3dVector([color] * (len(pts) - 1))
        return [ls]

    @staticmethod
    def _fit_line_svd_as_points(pts, n_samples=30):
        arr = np.asarray(pts)
        if arr.shape[0] < 2:
            return arr
        center = arr.mean(axis=0)
        _, _, vh = np.linalg.svd(arr - center, full_matrices=False)
        d = vh[0] / np.linalg.norm(vh[0])
        proj = np.dot(arr - center, d)
        ts = np.linspace(proj.min(), proj.max(), n_samples)
        return np.array([center + t * d for t in ts])

    @staticmethod
    def _rotate_vector_around_axis(v, k, theta):
        k = k / np.linalg.norm(k)
        return (
            v * np.cos(theta)
            + np.cross(k, v) * np.sin(theta)
            + k * np.dot(k, v) * (1 - np.cos(theta))
        )

    def _create_arc_points(self, center, end1, end2, n=30):
        v1, v2 = end1 - center, end2 - center
        v1u, v2u = v1 / np.linalg.norm(v1), v2 / np.linalg.norm(v2)
        angle = np.arccos(np.clip(v1u.dot(v2u), -1, 1))
        axis = np.cross(v1u, v2u)
        if np.linalg.norm(axis) < 1e-12:
            return np.vstack([center, end1, end2])
        return np.array(
            [
                center + self._rotate_vector_around_axis(v1, axis, t)
                for t in np.linspace(0, angle, n + 1)
            ]
        )

    @staticmethod
    def _create_sphere_at_point(center, radius=0.002, color=[0, 0, 0]):
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius)
        mesh.compute_vertex_normals()
        mesh.translate(center)
        mesh.paint_uniform_color(color)
        return mesh

    def _arc_for_angle(self, stem_start, stem_end, leaf_end, samples=30):
        arc_pts = self._create_arc_points(stem_start, stem_end, leaf_end, samples)
        if len(arc_pts) < 2:
            return []
        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(arc_pts)
        ls.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(len(arc_pts) - 1)])
        ls.colors = o3d.utility.Vector3dVector([[0, 0, 0]] * (len(arc_pts) - 1))
        sphere = self._create_sphere_at_point(arc_pts[len(arc_pts) // 2])
        return [ls, sphere]