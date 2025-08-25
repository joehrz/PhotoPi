# PhotoPi/photopack/point_cloud_analysis/main.py

import argparse
import open3d as o3d
import numpy as np
import logging
import sys
import os
from pathlib import Path
import csv
import json
import re

# SVG visualization utilities
try:
    from utils.svg_utils import SVGRenderer
    SVG_AVAILABLE = True
except ImportError:
    SVG_AVAILABLE = False
    logging.warning("SVG utilities not available. Install 'svgwrite' for SVG export functionality.")

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


###############################################################################
# 1) Read Metrics Config
###############################################################################

def load_metrics_config(json_path="metrics_config.json"):
    """
    Loads the base headers from a JSON file.
    Example:
      {
        "headers": ["Cultivar", "H_Max", ..., "Leaf_angle_count"]
      }
    """
    if not os.path.exists(json_path):
        logger.error(f"Metrics config JSON not found: {json_path}")
        raise FileNotFoundError(f"{json_path} not found")
    
    with open(json_path, "r") as f:
        config_data = json.load(f)
    return config_data.get("headers", [])


###############################################################################
# 2) CSV Helper with Dynamic Columns
###############################################################################

GLOBAL_HEADERS = None
CSV_PATH = None

def init_csv(csv_path, base_headers):
    """
    Initialize a global set of headers from base_headers + any existing CSV columns.
    """
    global GLOBAL_HEADERS, CSV_PATH
    CSV_PATH = csv_path
    GLOBAL_HEADERS = set(base_headers)

    if os.path.exists(csv_path):
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            existing = next(reader, None)
            if existing:
                GLOBAL_HEADERS |= set(existing)
    else:
        logger.info(f"CSV {csv_path} does not exist yet; will create when appending rows.")


def append_to_csv(row_data):
    """
    Inserts or updates a row in the CSV file. If a row with the same 'Cultivar'
    already exists, it will be overwritten.
    """
    global GLOBAL_HEADERS, CSV_PATH

    GLOBAL_HEADERS |= set(row_data.keys())
    all_headers = sorted(GLOBAL_HEADERS)

    rows = []
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            rows = list(reader)

    cultivar = row_data.get("Cultivar")
    updated = False
    for i, row in enumerate(rows):
        if row.get("Cultivar") == cultivar:
            rows[i] = {col: row_data.get(col, row.get(col, "")) for col in all_headers}
            updated = True
            break

    if not updated:
        rows.append({col: row_data.get(col, "") for col in all_headers})

    with open(CSV_PATH, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=all_headers)
        writer.writeheader()
        for row in rows:
            writer.writerow({col: row.get(col, "") for col in all_headers})

    logger.info(f"Inserted/Updated row for {cultivar}")


###############################################################################
# 3) Load + Process .ply
###############################################################################

def load_point_cloud(filename):
    """
    Load a .ply with Open3D.
    """
    try:
        pcd = o3d.io.read_point_cloud(str(filename))
        if not pcd.has_points():
            logger.error(f"No points found: {filename}")
            raise ValueError(f"No points in {filename}")
        logger.info(f"Loaded point cloud: {filename}")
        return pcd
    except Exception as e:
        logger.error(f"Error loading point cloud: {e}")
        raise


def safe_div(a, b):
    """Return a/b if b is non-zero, else empty string."""
    return a / b if b else ""


def compute_metrics(point_cloud, modules_to_run, seg_mode, scale, output_path, base, enable_svg=False, svg_camera_params=None):
    """
    Call each module (convex_hull, hr_analysis, leaf_angles, projection, etc.)
    Return a dict with all relevant metrics.
    
    Args:
        point_cloud: Open3D point cloud object
        modules_to_run: List of module names to execute
        seg_mode: Segmentation mode ('auto', 'manual', 'both')
        scale: Scale factor for measurements
        output_path: Path for output files
        base: Base filename for outputs
        enable_svg: Whether to generate SVG visualizations
        svg_camera_params: Tuple of (camera_matrix, extrinsic_matrix) for SVG projection
    """
    row_data = {}
    svg_renderer = None
    
    # Initialize SVG renderer if enabled
    if enable_svg and SVG_AVAILABLE:
        svg_renderer = SVGRenderer(width=1200, height=900)
        if svg_camera_params is None:
            # Use default camera parameters if not provided
            camera_matrix = np.array([
                [800, 0, 600],
                [0, 800, 450], 
                [0, 0, 1]
            ], dtype=float)
            extrinsic_matrix = np.eye(4)
            extrinsic_matrix[2, 3] = -50  # Move camera back
            svg_camera_params = (camera_matrix, extrinsic_matrix)
            logger.info("Using default camera parameters for SVG export")
        
        # Determine SVG output directory
        svg_output_path = output_path if output_path else os.path.join(os.path.dirname(__file__), "..", "..")
        logger.info(f"SVG export enabled. Output path: {svg_output_path}")

    # 1) PROCESSING
    if 'processing' in modules_to_run:
        try:
            from point_cloud.processing import PointCloudProcessor
            proc = PointCloudProcessor(point_cloud)
            proc.process()
            
            # SVG export for processing
            if svg_renderer:
                try:
                    points = np.asarray(point_cloud.points)
                    svg_filename = os.path.join(svg_output_path, f"{base}_processing.svg")
                    svg_renderer.export_point_cloud_svg(
                        points, svg_camera_params[0], svg_camera_params[1],
                        filename=svg_filename,
                        color_scheme="uniform",
                        point_radius=1.0
                    )
                    logger.info(f"[processing] SVG exported: {svg_filename}")
                except Exception as e:
                    logger.warning(f"[processing] SVG export failed: {e}")
            
            logger.info("Finished 'processing' module.")
        except ImportError as e:
            logger.error(f"Could not import processing module: {e}")

    # 2) CONVEX HULL
    if 'convex_hull' in modules_to_run:
        try:
            #from photopack.point_cloud_analysis.point_cloud.convex_hull import ConvexHullAnalyzer
            from point_cloud.convex_hull import ConvexHullAnalyzer
            hull = ConvexHullAnalyzer(point_cloud)
            v100_u = hull.compute_convex_hull_volume(scale=1.0)
            v100   = hull.compute_convex_hull_volume(scale=scale)
            v60_u  = hull.process_pc_percentile_volume(60, scale=1.0, visualize=False)
            v60    = hull.process_pc_percentile_volume(60, scale=scale, visualize=False)
            v40_u  = hull.process_pc_percentile_volume(40, scale=1.0, visualize=False)
            v40    = hull.process_pc_percentile_volume(40, scale=scale, visualize=False)

            row_data.update({
                "V_100_unscaled": v100_u,
                "V_100":         v100,
                "V_60_unscaled": v60_u,
                "V_60":          v60,
                "V_40_unscaled": v40_u,
                "V_40":          v40,
            })

            hull.visualize_convex_hull()
            
            # SVG export for convex hull
            if svg_renderer:
                try:
                    points = np.asarray(point_cloud.points)
                    svg_filename = os.path.join(svg_output_path, f"{base}_convex_hull.svg")
                    svg_renderer.export_point_cloud_svg(
                        points, svg_camera_params[0], svg_camera_params[1],
                        filename=svg_filename,
                        color_scheme="height",
                        point_radius=1.5
                    )
                    logger.info(f"[convex_hull] SVG exported: {svg_filename}")
                except Exception as e:
                    logger.warning(f"[convex_hull] SVG export failed: {e}")
            
            logger.info(
                f"[convex_hull] unscaled={v100_u:.2f}, scaled={v100:.2f}, "
                f"top60_u={v60_u:.2f}, top60={v60:.2f}, "
                f"top40_u={v40_u:.2f}, top40={v40:.2f}"
            )
        except ImportError as e:
            logger.error(f"Could not import convex_hull module: {e}")

    # 3) HR ANALYSIS
    if 'hr_analysis' in modules_to_run:
        try:
            from point_cloud.hr_analysis import HRAnalyzer
            hr = HRAnalyzer(point_cloud, scale)
            hr.analyze_hr_with_mainstem(
                alpha=1.0, beta=2.0,
                raindrop_alpha=1.0, raindrop_beta=1.0,
                gamma=10.0, delta=5.0,
                use_trunk_axis=True, debug=True
            )
            hr.visualize_with_open3d()
            hr.plot_radius_vs_theta()

            # SVG export for HR analysis
            if svg_renderer:
                try:
                    points = np.asarray(point_cloud.points)
                    svg_filename = os.path.join(svg_output_path, f"{base}_hr_analysis.svg")
                    svg_renderer.export_point_cloud_svg(
                        points, svg_camera_params[0], svg_camera_params[1],
                        filename=svg_filename,
                        color_scheme="height",
                        point_radius=1.0
                    )
                    logger.info(f"[hr_analysis] SVG exported: {svg_filename}")
                except Exception as e:
                    logger.warning(f"[hr_analysis] SVG export failed: {e}")

            row_data.update({
                "H_Max":             hr.height,
                "H_Max_unscaled":    hr.height_unscaled,
                "R_Max_unscaled":    hr.max_radius_unscaled,
                "R_Max":             hr.max_radius,
                "H_over_R":          safe_div(hr.height, hr.max_radius)
            })
            logger.info(
                f"[hr_analysis] H_Max={hr.height:.2f}, R_Max={hr.max_radius:.2f}, "
                f"H_over_R={row_data['H_over_R']}"
            )
        except ImportError as e:
            logger.error(f"Could not import hr_analysis module: {e}")

    # 4) PROJECTION
    if 'projection' in modules_to_run:
        try:
            from point_cloud.projection import ProjectionAnalyzer
            proj = ProjectionAnalyzer(point_cloud, scale)

            area = proj.compute_alpha_shape_area(alpha=15, apply_scale=True)
            area_unit = area / (scale**2) if scale else area

            row_data.update({
                "plane_projection_area":          area,
                "plane_projection_area_unitless": area_unit,
            })
            proj.plot_original_points()
            proj.plot_alpha_shape()
            proj.plot_points_and_alpha_shape()
            
            # SVG export for projection analysis
            if svg_renderer:
                try:
                    points = np.asarray(point_cloud.points)
                    svg_filename = os.path.join(svg_output_path, f"{base}_projection.svg")
                    svg_renderer.export_point_cloud_svg(
                        points, svg_camera_params[0], svg_camera_params[1],
                        filename=svg_filename,
                        color_scheme="distance",
                        point_radius=1.0
                    )
                    logger.info(f"[projection] SVG exported: {svg_filename}")
                except Exception as e:
                    logger.warning(f"[projection] SVG export failed: {e}")

            logger.info(
                f"[projection] plane_projection_area={area:.2f}, "
                f"unitless={area_unit:.2f}"
            )
        except ImportError as e:
            logger.error(f"Could not import projection module: {e}")

    # 5) SEGMENTATION
    if 'segmentation' in modules_to_run:
        try:
            from point_cloud.segmentation_extractor import Segmentation
            seg = Segmentation(point_cloud, min_cluster_size=50)
            if seg_mode in ('auto', 'both'):
                seg.visualize_downsampled()
                seg.preprocess(voxel_size=0.005, nb_neighbors=20, std_ratio=2.0)
                seg.graph_based_segmentation(k=50, z_scale=1.0, distance_threshold=0.009)
                seg.refine_with_mst_segmentation(num_cuts=2)
                seg.refine_clusters(min_cluster_size=30)
                seg.map_labels_to_original()
                seg.segment_and_store()
                logger.info("[segmentation] Auto-segmentation complete.")
                seg.visualize_segments_original()
                
                # SVG export for segmentation
                if svg_renderer:
                    try:
                        points = np.asarray(point_cloud.points)
                        svg_filename = os.path.join(svg_output_path, f"{base}_segmentation.svg")
                        svg_renderer.export_point_cloud_svg(
                            points, svg_camera_params[0], svg_camera_params[1],
                            filename=svg_filename,
                            color_scheme="uniform",
                            point_radius=1.2
                        )
                        logger.info(f"[segmentation] SVG exported: {svg_filename}")
                    except Exception as e:
                        logger.warning(f"[segmentation] SVG export failed: {e}")
                        
            if seg_mode in ('manual', 'both'):
                logger.info("[segmentation] Manual steps not fully shown here.")
        except ImportError as e:
            logger.error(f"Could not import segmentation module: {e}")

    # 6) MAIN STEM SEGMENTATION
    if 'mainstem_segmentation' in modules_to_run:
        try:
            from point_cloud.main_stem_segmentation import MainStemSegmentation
            segmentation = MainStemSegmentation(point_cloud)
            segmentation.run_full_pipeline(
                alpha=1.0, beta=1.0,
                raindrop_alpha=1.0, raindrop_beta=1.0,
                gamma=10.0, delta=2.0,
                use_trunk_axis=True, debug=True,
                output_path=output_path, base=base
            )
            # branch_data, pipeline_stats = segmentation.run_adaptive_pipeline(
            #     target_points_per_slice = 200,
            #     multi_scale_levels      = 3,
            #     use_geometric_bridging  = False,
            #     max_angle_deg           = 45,
            #     max_z_gap               = 5,
            #     density_tolerance       = 4.0,
            #     alpha                   = 1.0,
            #     beta                    = 0.5,
            #     raindrop_alpha          = 1.0,
            #     raindrop_beta           = 1.0,
            #     gamma                   = 2.0,
            #     delta                   = 1.0,
            #     use_trunk_axis          = True,
            #     debug                   = True,
            #     output_path             = output_path,
            #     base                    = base
            # )

            segmentation.visualize_final_graph_types()
            logger.info("[mainstem_segmentation] Pipeline complete.")

            if segmentation.labeled_pcd is not None:
                o3d.visualization.draw_geometries(
                    [segmentation.labeled_pcd],
                    window_name="MainStemSegmentation - Labeled PCD"
                )
                
                # SVG export for main stem segmentation
                if svg_renderer:
                    try:
                        points = np.asarray(segmentation.labeled_pcd.points)
                        colors = None
                        if segmentation.labeled_pcd.has_colors():
                            colors = np.asarray(segmentation.labeled_pcd.colors)
                        
                        svg_filename = os.path.join(svg_output_path, f"{base}_mainstem_segmentation.svg")
                        svg_renderer.export_point_cloud_svg(
                            points, svg_camera_params[0], svg_camera_params[1],
                            colors=colors,
                            filename=svg_filename,
                            color_scheme="original" if colors is not None else "height",
                            point_radius=1.0
                        )
                        logger.info(f"[mainstem_segmentation] SVG exported: {svg_filename}")
                    except Exception as e:
                        logger.warning(f"[mainstem_segmentation] SVG export failed: {e}")

            if 'leaf_angles' in modules_to_run:
                from point_cloud.leaf_angles import LeafAngleAnalyzer
                la = LeafAngleAnalyzer(segmentation)
                la.compute_leaf_angles_node_bfs(
                    n_main_stem=5, n_leaf=5,
                    min_leaf_for_angle=5, max_bfs_depth=5,
                    angle_mode="auto"
                )
                la.visualize_leaf_angles()
                for i, bd in enumerate(la.branch_data):
                    logger.info(
                        f"Branch {i:2d}  raw={bd['angle_raw']:.1f}°, "
                        f"acute={bd['angle_acute']:.1f}°, "
                        f"obtuse={bd['angle_obtuse']:.1f}°, "
                        f"final={bd['angle_final']:.1f}°"
                    )

                valid = [a for a in la.angles if a is not None]
                row_data["Main_leaf_angle"]   = la.main_leaf_angle or ""
                row_data["Leaf_angle_count"]  = len(valid)
                for i, a in enumerate(valid):
                    row_data[f"Leaf_angle_{i}"] = a

                # SVG export for leaf angles
                if svg_renderer and hasattr(la, 'branch_data'):
                    try:
                        # Export individual leaf angle visualization
                        svg_filename = os.path.join(svg_output_path, f"{base}_leaf_angles.svg")
                        svg_renderer.export_leaf_angles_svg(
                            la.branch_data, svg_camera_params[0], svg_camera_params[1],
                            filename=svg_filename,
                            stem_radius=4.0,
                            leaf_radius=4.0,
                            midpoint_radius=6.0
                        )
                        logger.info(f"[leaf_angles] SVG exported: {svg_filename}")
                        
                        # Export combined visualization with point cloud background
                        if segmentation.labeled_pcd is not None:
                            bg_points = np.asarray(segmentation.labeled_pcd.points)
                            combined_svg = os.path.join(svg_output_path, f"{base}_combined_analysis.svg")
                            svg_renderer.export_combined_svg(
                                point_cloud=bg_points,
                                branch_data=la.branch_data,
                                camera_matrix=svg_camera_params[0],
                                extrinsic_matrix=svg_camera_params[1],
                                filename=combined_svg,
                                point_radius=0.8,
                                angle_scale=1.2
                            )
                            logger.info(f"[leaf_angles] Combined SVG exported: {combined_svg}")
                            
                    except Exception as e:
                        logger.warning(f"[leaf_angles] SVG export failed: {e}")

                logger.info(f"[leaf_angles] Computed leaf angles => {la.angles}")
        except ImportError as e:
            logger.error(f"Could not import mainstem_segmentation module: {e}")

    # Derived metrics
    if "H_Max" in row_data and "V_100" in row_data:
        row_data["H_over_V"] = safe_div(row_data["H_Max"], row_data["V_100"])
    if "R_Max" in row_data and "V_100" in row_data:
        row_data["R_over_V"] = safe_div(row_data["R_Max"], row_data["V_100"])
    if "H_Max" in row_data and "V_40" in row_data:
        row_data["H_over_V_40"] = safe_div(row_data["H_Max"], row_data["V_40"])
    if "H_Max" in row_data and "V_60" in row_data:
        row_data["H_over_V_60"] = safe_div(row_data["H_Max"], row_data["V_60"])
    if "R_Max" in row_data and "V_40" in row_data:
        row_data["R_over_V_40"] = safe_div(row_data["R_Max"], row_data["V_40"])
    if "R_Max" in row_data and "V_60" in row_data:
        row_data["R_over_V_60"] = safe_div(row_data["R_Max"], row_data["V_60"])
    if "V_60" in row_data and "V_40" in row_data:
        row_data["V_60_over_V_40"] = safe_div(row_data["V_60"], row_data["V_40"])
    if "V_100" in row_data and "V_40" in row_data:
        row_data["V_over_V_40"] = safe_div(row_data["V_100"], row_data["V_40"])
    if "V_100" in row_data and "V_60" in row_data:
        row_data["V_over_V_60"] = safe_div(row_data["V_100"], row_data["V_60"])

    row_data["Scale"] = scale
    logger.info("[compute_metrics] Final row_data:")
    for k, v in row_data.items():
        logger.info(f"  {k} = {v}")

    return row_data


def process_single_file(ply_file, modules_to_run, seg_mode, scale_map, output_path, enable_svg=False, interactive_camera=False):
    """
    Perform the actual pipeline on one .ply file, returning a dict of metrics.
    
    Args:
        ply_file: Path to the .ply file
        modules_to_run: List of module names to execute
        seg_mode: Segmentation mode
        scale_map: Dictionary mapping cultivar names to scale values
        output_path: Output directory path
        enable_svg: Whether to generate SVG visualizations
        interactive_camera: Whether to capture camera parameters interactively
    """
    base = Path(ply_file).stem
    cultivar_key = re.sub(r"_fused_output_cluster_\d+$", "", base)
    scale = scale_map.get(cultivar_key)
    if scale is None:
        logger.error(f"No scale value for '{cultivar_key}' in scale JSON")
        return {}

    pcd = load_point_cloud(ply_file)
    
    # Handle SVG camera parameters
    svg_camera_params = None
    if enable_svg and SVG_AVAILABLE:
        if interactive_camera:
            try:
                logger.info("Capturing camera parameters interactively...")
                camera_matrix, extrinsic_matrix = SVGRenderer.capture_open3d_camera(
                    [pcd], f"Camera Setup - {base}"
                )
                svg_camera_params = (camera_matrix, extrinsic_matrix)
                logger.info("Camera parameters captured successfully")
            except Exception as e:
                logger.warning(f"Interactive camera capture failed: {e}. Using defaults.")
                svg_camera_params = None
    
    row = compute_metrics(pcd, modules_to_run, seg_mode, scale, output_path, base, enable_svg, svg_camera_params)
    row["Cultivar"] = base
    return row


def main():
    parser = argparse.ArgumentParser(
        description='Full pipeline with JSON config, multi leaf angles, logging metrics.'
    )
    parser.add_argument('path', help='A .ply file OR a directory of .ply files.')
    parser.add_argument('--output', help='Path to output')
    parser.add_argument('--diameter', type=float, default=1.3, help='Ring diameter in cm.')
    parser.add_argument(
        '--module', choices=[
            'processing','convex_hull','hr_analysis','leaf_angles',
            'projection','segmentation','mainstem_segmentation','all'
        ],
        nargs='+', help='Modules to run.'
    )
    parser.add_argument(
        '--segmentation-mode', choices=['auto','manual','both'],
        default='auto', help='How to run segmentation.'
    )
    parser.add_argument('--json_config', default='metrics_config.json',
                        help='Path to JSON with base headers.')
    parser.add_argument('--json_scale', default='scale_values.json',
                        help='Path to JSON with point cloud scale values.')
    parser.add_argument('--csv-out', default='analysis_metrics.csv',
                        help='Output CSV file.')
    parser.add_argument('--enable-svg', action='store_true',
                        help='Enable SVG visualization export (requires svgwrite)')
    parser.add_argument('--interactive-camera', action='store_true', 
                        help='Interactively capture camera parameters for SVG projection')

    args = parser.parse_args()

    # Prepare modules list
    available = ['processing','convex_hull','hr_analysis','leaf_angles',
                 'projection','segmentation','mainstem_segmentation']
    modules = args.module or available
    if 'all' in modules:
        modules = available

    # Load configs once
    base_headers = load_metrics_config(args.json_config)
    with open(args.json_scale, 'r') as f:
        scale_map = json.load(f)

    init_csv(args.csv_out, base_headers)

    path = Path(args.path)
    if path.is_file():
        ply_files = [path]
    elif path.is_dir():
        ply_files = list(path.glob('*.ply'))
        if not ply_files:
            logger.warning(f"No .ply found in {path}")
    else:
        logger.error(f"Invalid path: {path}")
        sys.exit(1)

    # Check SVG prerequisites
    if args.enable_svg and not SVG_AVAILABLE:
        logger.error("SVG export requested but svgwrite not available. Install with: pip install svgwrite")
        sys.exit(1)
    
    # Ensure output directory exists for SVG files
    if args.enable_svg:
        svg_output_dir = args.output if args.output else os.path.join(os.path.dirname(__file__), "..", "..")
        os.makedirs(svg_output_dir, exist_ok=True)
        logger.info(f"SVG output directory: {svg_output_dir}")
    
    for ply in ply_files:
        row = process_single_file(
            str(ply), modules, args.segmentation_mode, scale_map, args.output,
            enable_svg=args.enable_svg, interactive_camera=args.interactive_camera
        )
        if row:
            append_to_csv(row)

    logger.info("All processing completed.")
    
    # Summarize SVG outputs if enabled
    if args.enable_svg:
        svg_check_dir = args.output if args.output else os.path.join(os.path.dirname(__file__), "..", "..")
        if os.path.exists(svg_check_dir):
            svg_files = [f for f in os.listdir(svg_check_dir) if f.endswith('.svg')]
            if svg_files:
                logger.info(f"\nGenerated {len(svg_files)} SVG files:")
                for svg_file in sorted(svg_files):
                    logger.info(f"  - {svg_file}")
                logger.info(f"\nSVG files saved to: {svg_check_dir}")
            else:
                logger.info("No SVG files were generated (check for errors above)")


if __name__ == '__main__':
    main()
