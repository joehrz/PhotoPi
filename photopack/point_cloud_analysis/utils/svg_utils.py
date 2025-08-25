import numpy as np
import svgwrite
import open3d as o3d
from typing import Optional, Tuple, List, Dict, Any, Union
import colorsys
import os


class SVGRenderer:
    """
    SVG rendering utility for point cloud analysis visualization.
    
    Provides methods to create SVG visualizations of:
    - Point clouds with various coloring schemes
    - Leaf angle measurements and annotations
    - Graph structures from point cloud analysis
    - Combined visualizations with overlays
    
    Attributes:
        width (int): SVG canvas width in pixels
        height (int): SVG canvas height in pixels
        background_color (str): Background color for SVG canvas
    """
    
    def __init__(self, width: int = 800, height: int = 600, background_color: str = "white"):
        self.width = width
        self.height = height
        self.background_color = background_color
        
    def project_points(self, points_3d: np.ndarray, camera_matrix: np.ndarray, 
                      extrinsic_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Project 3D points to 2D screen coordinates.
        
        Args:
            points_3d: Nx3 array of 3D points
            camera_matrix: 3x3 intrinsic camera matrix
            extrinsic_matrix: 4x4 extrinsic transformation matrix
            
        Returns:
            Tuple of (projected_2d_points, visibility_mask)
        """
        if len(points_3d) == 0:
            return np.array([]).reshape(0, 2), np.array([], dtype=bool)
            
        # Convert to homogeneous coordinates
        points_hom = np.hstack([points_3d, np.ones((len(points_3d), 1))])
        
        # Apply extrinsic transformation
        camera_coords = (extrinsic_matrix @ points_hom.T).T
        
        # Check which points are in front of camera
        visibility_mask = camera_coords[:, 2] > 0
        
        if not np.any(visibility_mask):
            return np.array([]).reshape(0, 2), visibility_mask
            
        # Project visible points
        visible_points = camera_coords[visibility_mask, :3]
        projected = (camera_matrix @ visible_points.T)
        projected = projected[:2] / projected[2]  # Normalize by z-coordinate
        
        # Create output array with all points
        projected_2d = np.zeros((len(points_3d), 2))
        projected_2d[visibility_mask] = projected.T
        
        return projected_2d, visibility_mask
        
    def fit_line_svd_as_points(self, points: np.ndarray, n_points: int = 30) -> np.ndarray:
        """
        Fit a line through points using SVD and return points along the line.
        
        Args:
            points: Nx3 array of input points
            n_points: Number of points to generate along the fitted line
            
        Returns:
            n_points x 3 array of points along the fitted line
        """
        points_arr = np.asarray(points, dtype=float)
        if len(points_arr) < 2:
            return points_arr
            
        center = points_arr.mean(axis=0)
        _, _, vh = np.linalg.svd(points_arr - center, full_matrices=False)
        direction = vh[0] / np.linalg.norm(vh[0])
        
        # Project points onto the line to get parameter range
        projections = (points_arr - center) @ direction
        t_min, t_max = projections.min(), projections.max()
        
        # Generate points along the line
        t_values = np.linspace(t_min, t_max, n_points)
        line_points = np.array([center + t * direction for t in t_values])
        
        return line_points
        
    def create_arc_points(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, 
                         n_points: int = 30) -> np.ndarray:
        """
        Create arc points between three 3D points for angle visualization.
        
        Args:
            p1: Starting point of first vector
            p2: Common point (vertex of angle)
            p3: End point of second vector
            n_points: Number of points in the arc
            
        Returns:
            Array of 3D points forming an arc
        """
        v1 = p2 - p1
        v2 = p3 - p1
        
        norm1, norm2 = np.linalg.norm(v1), np.linalg.norm(v2)
        
        if norm1 < 1e-6 or norm2 < 1e-6:
            return np.vstack([p1, p2])
            
        u1, u2 = v1 / norm1, v2 / norm2
        angle = np.arccos(np.clip(u1 @ u2, -1, 1))
        
        # Create rotation axis
        axis = np.cross(u1, u2)
        axis_norm = np.linalg.norm(axis)
        
        if axis_norm < 1e-6:  # Vectors are parallel
            return np.vstack([p1, p2])
            
        axis = axis / axis_norm
        
        # Generate arc points using Rodrigues' rotation formula
        arc_points = []
        for i in range(n_points + 1):
            t = angle * i / n_points
            # Rodrigues' rotation formula
            v_rot = (v1 * np.cos(t) + 
                    np.cross(axis, v1) * np.sin(t) + 
                    axis * (axis @ v1) * (1 - np.cos(t)))
            arc_points.append(p1 + v_rot)
            
        return np.array(arc_points)
        
    def export_point_cloud_svg(self, points: np.ndarray, 
                              camera_matrix: np.ndarray,
                              extrinsic_matrix: np.ndarray,
                              colors: Optional[np.ndarray] = None,
                              filename: str = "point_cloud.svg",
                              point_radius: float = 1.0,
                              color_scheme: str = "height") -> str:
        """
        Export point cloud as SVG visualization.
        
        Args:
            points: Nx3 array of 3D points
            camera_matrix: 3x3 camera intrinsic matrix
            extrinsic_matrix: 4x4 camera extrinsic matrix
            colors: Optional Nx3 RGB colors (0-1 range)
            filename: Output SVG filename
            point_radius: Radius of point circles
            color_scheme: Color scheme ('height', 'uniform', 'original')
            
        Returns:
            Path to saved SVG file
        """
        dwg = svgwrite.Drawing(filename, size=(self.width, self.height))
        dwg.add(dwg.rect((0, 0), (self.width, self.height), fill=self.background_color))
        
        if len(points) == 0:
            dwg.save()
            return filename
            
        # Project points to 2D
        points_2d, visibility_mask = self.project_points(points, camera_matrix, extrinsic_matrix)
        visible_points = points[visibility_mask]
        visible_points_2d = points_2d[visibility_mask]
        
        if len(visible_points) == 0:
            dwg.save()
            return filename
            
        # Determine colors
        if colors is not None and len(colors) == len(points):
            point_colors = colors[visibility_mask]
            # Convert to 0-255 range if in 0-1 range
            if np.max(point_colors) <= 1.0:
                point_colors = (point_colors * 255).astype(int)
        else:
            point_colors = self._generate_colors(visible_points, color_scheme)
            
        # Draw points
        for i, (x, y) in enumerate(visible_points_2d):
            if 0 <= x <= self.width and 0 <= y <= self.height:
                color_rgb = point_colors[i] if len(point_colors) > i else [128, 128, 128]
                color_hex = f"rgb({color_rgb[0]},{color_rgb[1]},{color_rgb[2]})"
                
                dwg.add(dwg.circle(
                    center=(x, y),
                    r=point_radius,
                    fill=color_hex,
                    stroke="none"
                ))
        
        dwg.save()
        return filename
        
    def export_leaf_angles_svg(self, branch_data: List[Dict[str, Any]],
                              camera_matrix: np.ndarray,
                              extrinsic_matrix: np.ndarray,
                              filename: str = "leaf_angles.svg",
                              stem_radius: float = 6.0,
                              leaf_radius: float = 6.0,
                              midpoint_radius: float = 8.0) -> str:
        """
        Export leaf angle analysis as SVG with stem/leaf points and angle arcs.
        
        Args:
            branch_data: List of dictionaries containing stem_points, leaf_points, and angle data
            camera_matrix: 3x3 camera intrinsic matrix
            extrinsic_matrix: 4x4 camera extrinsic matrix
            filename: Output SVG filename
            stem_radius: Radius for stem point circles
            leaf_radius: Radius for leaf point circles
            midpoint_radius: Radius for angle midpoint circles
            
        Returns:
            Path to saved SVG file
        """
        dwg = svgwrite.Drawing(filename, size=(self.width, self.height))
        dwg.add(dwg.rect((0, 0), (self.width, self.height), fill=self.background_color))
        
        for branch in branch_data:
            if 'stem_points' not in branch or 'leaf_points' not in branch:
                continue
                
            stem_points = np.asarray(branch['stem_points'])
            leaf_points = np.asarray(branch['leaf_points'])
            
            # Draw stem and leaf points
            point_sets = [
                (stem_points, "red", stem_radius),
                (leaf_points, "green", leaf_radius)
            ]
            
            for points, color, radius in point_sets:
                if len(points) > 0:
                    points_2d, visibility_mask = self.project_points(points, camera_matrix, extrinsic_matrix)
                    visible_2d = points_2d[visibility_mask]
                    
                    for x, y in visible_2d:
                        if 0 <= x <= self.width and 0 <= y <= self.height:
                            dwg.add(dwg.circle(
                                center=(x, y),
                                r=radius,
                                fill=color,
                                stroke="none"
                            ))
            
            # Draw regression lines
            if len(stem_points) >= 2 and len(leaf_points) >= 2:
                stem_line = self.fit_line_svd_as_points(stem_points)
                leaf_line = self.fit_line_svd_as_points(leaf_points)
                
                line_sets = [
                    (stem_line, "blue"),
                    (leaf_line, "magenta")
                ]
                
                for line_points, color in line_sets:
                    line_2d, visibility_mask = self.project_points(line_points, camera_matrix, extrinsic_matrix)
                    if np.any(visibility_mask):
                        visible_line = line_2d[visibility_mask]
                        if len(visible_line) >= 2:
                            polyline_points = [tuple(p) for p in visible_line]
                            dwg.add(dwg.polyline(
                                points=polyline_points,
                                stroke=color,
                                fill="none",
                                stroke_width=2
                            ))
                
                # Draw angle arc
                if len(stem_line) >= 2 and len(leaf_line) >= 1:
                    s0, s1 = stem_line[0], stem_line[-1]
                    l1 = leaf_line[-1]
                    
                    arc_points = self.create_arc_points(s0, s1, l1)
                    arc_2d, visibility_mask = self.project_points(arc_points, camera_matrix, extrinsic_matrix)
                    
                    if np.any(visibility_mask):
                        visible_arc = arc_2d[visibility_mask]
                        if len(visible_arc) >= 2:
                            arc_polyline = [tuple(p) for p in visible_arc]
                            dwg.add(dwg.polyline(
                                points=arc_polyline,
                                stroke="black",
                                fill="none",
                                stroke_width=2
                            ))
                            
                            # Draw midpoint
                            if len(arc_points) > 0:
                                mid_idx = len(arc_points) // 2
                                mid_point = arc_points[mid_idx:mid_idx+1]
                                mid_2d, mid_vis = self.project_points(mid_point, camera_matrix, extrinsic_matrix)
                                
                                if np.any(mid_vis):
                                    x, y = mid_2d[0]
                                    if 0 <= x <= self.width and 0 <= y <= self.height:
                                        dwg.add(dwg.circle(
                                            center=(x, y),
                                            r=midpoint_radius,
                                            fill="black",
                                            stroke="none"
                                        ))
        
        dwg.save()
        return filename
        
    def export_graph_svg(self, graph_nodes: np.ndarray,
                        graph_edges: List[Tuple[int, int]],
                        node_types: Optional[Dict[int, str]] = None,
                        camera_matrix: np.ndarray = None,
                        extrinsic_matrix: np.ndarray = None,
                        filename: str = "graph.svg",
                        node_radius: float = 6.0,
                        line_width: float = 2.0) -> str:
        """
        Export graph structure as SVG visualization.
        
        Args:
            graph_nodes: Nx3 array of node positions
            graph_edges: List of (node_i, node_j) edge pairs
            node_types: Optional dictionary mapping node indices to type strings
            camera_matrix: 3x3 camera intrinsic matrix
            extrinsic_matrix: 4x4 camera extrinsic matrix
            filename: Output SVG filename
            node_radius: Radius for node circles
            line_width: Width of edge lines
            
        Returns:
            Path to saved SVG file
        """
        dwg = svgwrite.Drawing(filename, size=(self.width, self.height))
        dwg.add(dwg.rect((0, 0), (self.width, self.height), fill=self.background_color))
        
        if len(graph_nodes) == 0:
            dwg.save()
            return filename
            
        # Project nodes to 2D
        nodes_2d, visibility_mask = self.project_points(graph_nodes, camera_matrix, extrinsic_matrix)
        
        # Color mapping for node types
        type_colors = {
            'stem': 'red',
            'branch_off': 'blue', 
            'leaf': 'green',
            'unknown': 'gray'
        }
        
        # Draw edges first (so they appear behind nodes)
        for node_i, node_j in graph_edges:
            if (node_i < len(visibility_mask) and node_j < len(visibility_mask) and
                visibility_mask[node_i] and visibility_mask[node_j]):
                
                x1, y1 = nodes_2d[node_i]
                x2, y2 = nodes_2d[node_j]
                
                # Check if both points are within canvas bounds
                if (0 <= x1 <= self.width and 0 <= y1 <= self.height and
                    0 <= x2 <= self.width and 0 <= y2 <= self.height):
                    
                    dwg.add(dwg.line(
                        (x1, y1), (x2, y2),
                        stroke="black",
                        stroke_width=line_width
                    ))
        
        # Draw nodes
        for i, (x, y) in enumerate(nodes_2d):
            if visibility_mask[i] and 0 <= x <= self.width and 0 <= y <= self.height:
                node_type = node_types.get(i, 'unknown') if node_types else 'unknown'
                color = type_colors.get(node_type, 'gray')
                
                dwg.add(dwg.circle(
                    center=(x, y),
                    r=node_radius,
                    fill=color,
                    stroke="none"
                ))
        
        dwg.save()
        return filename
        
    def export_combined_svg(self, point_cloud: np.ndarray,
                           branch_data: List[Dict[str, Any]] = None,
                           graph_nodes: np.ndarray = None,
                           graph_edges: List[Tuple[int, int]] = None,
                           camera_matrix: np.ndarray = None,
                           extrinsic_matrix: np.ndarray = None,
                           filename: str = "combined_analysis.svg",
                           point_radius: float = 1.0,
                           angle_scale: float = 1.0) -> str:
        """
        Export combined visualization with point cloud background and analysis overlays.
        
        Args:
            point_cloud: Nx3 array of background point cloud
            branch_data: Optional leaf angle analysis data
            graph_nodes: Optional graph node positions
            graph_edges: Optional graph edge connections
            camera_matrix: 3x3 camera intrinsic matrix
            extrinsic_matrix: 4x4 camera extrinsic matrix
            filename: Output SVG filename
            point_radius: Radius for point cloud points
            angle_scale: Scale factor for angle visualization elements
            
        Returns:
            Path to saved SVG file
        """
        dwg = svgwrite.Drawing(filename, size=(self.width, self.height))
        dwg.add(dwg.rect((0, 0), (self.width, self.height), fill=self.background_color))
        
        # Draw background point cloud
        if len(point_cloud) > 0:
            points_2d, visibility_mask = self.project_points(point_cloud, camera_matrix, extrinsic_matrix)
            visible_points = point_cloud[visibility_mask]
            visible_2d = points_2d[visibility_mask]
            
            # Use height-based coloring for background
            colors = self._generate_colors(visible_points, "height")
            
            for i, (x, y) in enumerate(visible_2d):
                if 0 <= x <= self.width and 0 <= y <= self.height and i < len(colors):
                    color_rgb = colors[i]
                    color_hex = f"rgb({color_rgb[0]},{color_rgb[1]},{color_rgb[2]})"
                    
                    dwg.add(dwg.circle(
                        center=(x, y),
                        r=point_radius,
                        fill=color_hex,
                        stroke="none",
                        opacity=0.7  # Make background semi-transparent
                    ))
        
        # Draw graph structure if provided
        if graph_nodes is not None and len(graph_nodes) > 0:
            # This is a simplified version - you can expand with node types
            nodes_2d, node_vis = self.project_points(graph_nodes, camera_matrix, extrinsic_matrix)
            
            # Draw edges
            if graph_edges:
                for node_i, node_j in graph_edges:
                    if (node_i < len(node_vis) and node_j < len(node_vis) and
                        node_vis[node_i] and node_vis[node_j]):
                        
                        x1, y1 = nodes_2d[node_i]
                        x2, y2 = nodes_2d[node_j]
                        
                        dwg.add(dwg.line(
                            (x1, y1), (x2, y2),
                            stroke="black",
                            stroke_width=2
                        ))
            
            # Draw nodes
            for i, (x, y) in enumerate(nodes_2d):
                if node_vis[i] and 0 <= x <= self.width and 0 <= y <= self.height:
                    dwg.add(dwg.circle(
                        center=(x, y),
                        r=4,
                        fill="blue",
                        stroke="white",
                        stroke_width=1
                    ))
        
        # Draw leaf angle analysis if provided
        if branch_data:
            for branch in branch_data:
                if 'stem_points' not in branch or 'leaf_points' not in branch:
                    continue
                    
                stem_points = np.asarray(branch['stem_points'])
                leaf_points = np.asarray(branch['leaf_points'])
                
                # Draw angle visualization elements with scaling
                if len(stem_points) >= 2 and len(leaf_points) >= 2:
                    stem_line = self.fit_line_svd_as_points(stem_points)
                    leaf_line = self.fit_line_svd_as_points(leaf_points)
                    
                    # Draw regression lines
                    for line_points, color in [(stem_line, "blue"), (leaf_line, "magenta")]:
                        line_2d, vis_mask = self.project_points(line_points, camera_matrix, extrinsic_matrix)
                        if np.any(vis_mask):
                            visible_line = line_2d[vis_mask]
                            if len(visible_line) >= 2:
                                dwg.add(dwg.polyline(
                                    points=[tuple(p) for p in visible_line],
                                    stroke=color,
                                    fill="none",
                                    stroke_width=int(3 * angle_scale)
                                ))
                    
                    # Draw angle arc
                    if len(stem_line) >= 2 and len(leaf_line) >= 1:
                        s0, s1 = stem_line[0], stem_line[-1]
                        l1 = leaf_line[-1]
                        
                        arc_points = self.create_arc_points(s0, s1, l1)
                        arc_2d, arc_vis = self.project_points(arc_points, camera_matrix, extrinsic_matrix)
                        
                        if np.any(arc_vis) and len(arc_2d[arc_vis]) >= 2:
                            dwg.add(dwg.polyline(
                                points=[tuple(p) for p in arc_2d[arc_vis]],
                                stroke="black",
                                fill="none",
                                stroke_width=int(3 * angle_scale)
                            ))
        
        dwg.save()
        return filename
        
    def _generate_colors(self, points: np.ndarray, color_scheme: str) -> np.ndarray:
        """
        Generate colors for points based on the specified scheme.
        
        Args:
            points: Nx3 array of 3D points
            color_scheme: Color scheme ('height', 'uniform', 'distance')
            
        Returns:
            Nx3 array of RGB colors (0-255 range)
        """
        n_points = len(points)
        colors = np.zeros((n_points, 3), dtype=int)
        
        if color_scheme == "uniform":
            colors[:] = [100, 150, 200]  # Light blue
            
        elif color_scheme == "height":
            if n_points > 0:
                heights = points[:, 2]  # Z coordinate
                h_min, h_max = heights.min(), heights.max()
                
                if h_max > h_min:
                    normalized_heights = (heights - h_min) / (h_max - h_min)
                    
                    # Create height-based colormap (blue to red)
                    for i, h in enumerate(normalized_heights):
                        # Use HSV color space for smooth gradient
                        hue = (1.0 - h) * 0.7  # Blue (0.7) to red (0.0)
                        rgb = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
                        colors[i] = [int(c * 255) for c in rgb]
                else:
                    colors[:] = [128, 128, 128]  # Gray for uniform height
            
        elif color_scheme == "distance":
            if n_points > 0:
                center = points.mean(axis=0)
                distances = np.linalg.norm(points - center, axis=1)
                d_min, d_max = distances.min(), distances.max()
                
                if d_max > d_min:
                    normalized_distances = (distances - d_min) / (d_max - d_min)
                    
                    for i, d in enumerate(normalized_distances):
                        # Green to yellow to red based on distance
                        if d < 0.5:
                            # Green to yellow
                            colors[i] = [int(d * 2 * 255), 255, 0]
                        else:
                            # Yellow to red
                            colors[i] = [255, int((1 - d) * 2 * 255), 0]
                else:
                    colors[:] = [0, 255, 0]  # Green for uniform distance
        
        else:
            # Default to gray
            colors[:] = [128, 128, 128]
            
        return colors

    @staticmethod
    def capture_open3d_camera(geometries: List, window_name: str = "SVG Capture") -> Tuple[np.ndarray, np.ndarray]:
        """
        Capture camera parameters from Open3D viewer for SVG projection.
        
        Args:
            geometries: List of Open3D geometry objects to display
            window_name: Window title for the viewer
            
        Returns:
            Tuple of (camera_matrix, extrinsic_matrix)
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=window_name)
        
        for geom in geometries:
            vis.add_geometry(geom)
            
        print(f"[{window_name}] Adjust the view and close the window to capture camera parameters")
        vis.run()
        
        # Get camera parameters
        camera_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
        vis.destroy_window()
        
        intrinsic_matrix = camera_params.intrinsic.intrinsic_matrix
        extrinsic_matrix = camera_params.extrinsic
        
        return intrinsic_matrix, extrinsic_matrix


def create_example_visualization():
    """
    Example usage of the SVG renderer for point cloud analysis.
    """
    # Generate sample point cloud data
    n_points = 1000
    points = np.random.randn(n_points, 3) * 10
    points[:, 2] += 20  # Shift points away from camera
    
    # Create sample camera matrices (example values)
    camera_matrix = np.array([
        [800, 0, 400],
        [0, 800, 300], 
        [0, 0, 1]
    ], dtype=float)
    
    extrinsic_matrix = np.eye(4)
    extrinsic_matrix[2, 3] = -50  # Move camera back
    
    # Create renderer
    renderer = SVGRenderer(width=800, height=600)
    
    # Export point cloud with different color schemes
    renderer.export_point_cloud_svg(
        points, camera_matrix, extrinsic_matrix,
        filename="example_height_colors.svg",
        color_scheme="height"
    )
    
    renderer.export_point_cloud_svg(
        points, camera_matrix, extrinsic_matrix, 
        filename="example_uniform_colors.svg",
        color_scheme="uniform"
    )
    
    print("Example SVG files created: example_height_colors.svg, example_uniform_colors.svg")


if __name__ == "__main__":
    create_example_visualization()