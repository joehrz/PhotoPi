# PhotoPi/photopack/point_cloud_analysis/main.py

import argparse
import open3d as o3d
import numpy as np
import logging
import sys
import os
import glob
import csv
import json
import math


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
        sys.exit(1)
    
    with open(json_path, "r") as f:
        config_data = json.load(f)
    base_headers = config_data.get("headers", [])
    return base_headers

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

    # Start with base headers from JSON
    GLOBAL_HEADERS = set(base_headers)

    # If CSV exists, unify existing columns
    if os.path.exists(csv_path):
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            existing_headers = next(reader, None)
            if existing_headers:
                GLOBAL_HEADERS = set(existing_headers).union(GLOBAL_HEADERS)
    else:
        logger.info(f"CSV {csv_path} does not exist yet; will create when appending rows.")

def append_to_csv(row_data):
    """
    Inserts or updates a row in the CSV file. If a row with the same 'Cultivar'
    (i.e. file identifier) already exists, it will be overwritten.
    """
    global GLOBAL_HEADERS, CSV_PATH

    # Expand GLOBAL_HEADERS with any new keys from row_data
    for k in row_data.keys():
        GLOBAL_HEADERS.add(k)

    # Sort the final set of columns for consistent ordering
    all_headers = sorted(GLOBAL_HEADERS)

    # Read the CSV into a list of dictionaries (if it exists)
    rows = []
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)

    # Look for an existing row with the same 'Cultivar'
    cultivar = row_data.get("Cultivar")
    updated = False
    for i, row in enumerate(rows):
        if row.get("Cultivar") == cultivar:
            # Update this row with new data
            rows[i] = {col: row_data.get(col, row.get(col, "")) for col in all_headers}
            updated = True
            break
    if not updated:
        # Append as a new row if not found
        new_row = {col: row_data.get(col, "") for col in all_headers}
        rows.append(new_row)

    # Rewrite the CSV file with updated rows
    with open(CSV_PATH, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=all_headers)
        writer.writeheader()
        for row in rows:
            # Ensure all columns are present
            full_row = {col: row.get(col, "") for col in all_headers}
            writer.writerow(full_row)

    logger.info(f"Inserted/Updated row for {cultivar}")


###############################################################################
# 3) Load + Process .ply
###############################################################################

def load_point_cloud(filename):
    """
    Load a .ply with Open3D.
    """
    try:
        pcd = o3d.io.read_point_cloud(filename)
        if not pcd.has_points():
            logger.error(f"No points found: {filename}")
            sys.exit(1)
        logger.info(f"Loaded point cloud: {filename}")
        return pcd
    except Exception as e:
        logger.error(f"Error loading point cloud: {e}")
        sys.exit(1)

def compute_metrics(point_cloud, modules_to_run, seg_mode, scale, ring_diameter, top_percentage):
    """
    Actually call each module (convex_hull, hr_analysis, leaf_angles, projection, etc.)
    Return a dict with all relevant metrics. 
    We'll log (logger.info) the computed values for terminal output.
    """
    row_data = {}

    # 1) PROCESSING
    if 'processing' in modules_to_run:
        from photopack.point_cloud_analysis.point_cloud.processing import PointCloudProcessor
        proc = PointCloudProcessor(point_cloud)
        proc.process()
        point_cloud = proc.get_processed_point_cloud()
        logger.info("Finished 'processing' module.")

    # 2) CONVEX HULL
    if 'convex_hull' in modules_to_run:
        from photopack.point_cloud_analysis.point_cloud.convex_hull import ConvexHullAnalyzer
        hull = ConvexHullAnalyzer(point_cloud)

        v_100_unscaled = hull.compute_convex_hull_volume(scale=1.0)
        v_exclam = hull.compute_convex_hull_volume(scale=scale)
        v_60_unscaled = hull.process_pc_percentile_volume(60, scale=1.0, visualize=False)
        v_60 = hull.process_pc_percentile_volume(60, scale=scale, visualize=False)
        v_40_unscaled = hull.process_pc_percentile_volume(40, scale=1.0, visualize=False)
        v_40 = hull.process_pc_percentile_volume(40, scale=scale, visualize=False)

        row_data["V_100_unscaled"] = v_100_unscaled
        row_data["V_!00"] = v_exclam  # if this means scaled or unscaled
        row_data["V_60_unscaled"] = v_60_unscaled
        row_data["V_60"] = v_60
        row_data["V_40_unscaled"] = v_40_unscaled
        row_data["V_40"] = v_40

        logger.info(f"[convex_hull] unscaled={v_100_unscaled:.2f}, scaled={v_exclam:.2f}, top60_unscaled={v_60_unscaled:.2f}, top60_scaled={v_60:.2f}, top40_unscaled={v_40_unscaled:.2f}, top40_scaled={v_40:.2f}")

    # 3) HR ANALYSIS
    if 'hr_analysis' in modules_to_run:
        from photopack.point_cloud_analysis.point_cloud.hr_analysis import HRAnalyzer
        hr = HRAnalyzer(point_cloud, scale)
        hr.analyze_hr()

        row_data["H_Max"] = hr.height_density
        row_data["R_Max"] = hr.max_radius
        if hr.max_radius:
            row_data["H_over_R"] = hr.height_density / hr.max_radius
        else:
            row_data["H_over_R"] = ""

        logger.info(f"[hr_analysis] H_Max={hr.height_density:.2f}, R_Max={hr.max_radius:.2f}, H_over_R={row_data['H_over_R']}")



    # 5) PROJECTION
    if 'projection' in modules_to_run:
        from photopack.point_cloud_analysis.point_cloud.projection import ProjectionAnalyzer
        proj = ProjectionAnalyzer(point_cloud)
        area_scaled = proj.compute_alpha_shape_area(alpha=1.0)
        row_data["plane_projection_area"] = area_scaled
        row_data["plane_projection_area_unitless"] = area_scaled / (scale**2) if scale != 0 else area_scaled

        logger.info(f"[projection] plane_projection_area={area_scaled:.2f}, plane_projection_area_unitless={row_data['plane_projection_area_unitless']}")

    # 6) SEGMENTATION
    if 'segmentation' in modules_to_run:
        from photopack.point_cloud_analysis.point_cloud.segmentation_extractor import Segmentation
        seg = Segmentation(point_cloud, min_cluster_size=50)

        if seg_mode in ['auto','both']:
            seg.preprocess(voxel_size=0.005, nb_neighbors=20, std_ratio=2.0)
            seg.graph_based_segmentation(k=50, z_scale=1.0, distance_threshold=0.009)
            seg.refine_with_mst_segmentation(num_cuts=2)
            seg.refine_clusters(min_cluster_size=30)
            seg.map_labels_to_original()
            seg.segment_and_store()
            logger.info("[segmentation] Auto-segmentation complete.")

        if seg_mode in ['manual','both']:
            logger.info("[segmentation] Manual steps not fully shown here.")


    # 6) Main Stem SEGMENTATION
    if 'mainstem_segmentation' in modules_to_run:
        from photopack.point_cloud_analysis.point_cloud.main_stem_segmentation import MainStemSegmentation

        # Initialize the segmentation class.
        segmentation = MainStemSegmentation(point_cloud)
        # Run initial processing steps
        segmentation.process_point_cloud(eps=0.006, min_samples=50, n_sections=80)
        segmentation.aggregate_centroids()
        segmentation.build_graph(k=2)
        segmentation.remove_cycles()
        segmentation.visualize_graph_with_types()

        segmentation.segment_nodes()

        
        segmentation.visualize_graph_with_types()
        segmentation.bridge_graph_gap(max_gap=0.1)
        # Use the custom graph traversal to extract the main stem.
        main_stem_path = segmentation.traverse_main_stem(alpha=1.0, theta_threshold=math.pi/8)
    
        # Visualize the resulting main stem centerline.
        segmentation.visualize_centerline()
        segmentation.reclassify_nodes_by_centerline(distance_threshold=0.005)
        segmentation.visualize_graph_with_types()



        segmentation.label_branch_off_nodes(min_degree=3)
        segmentation.visualize_graph_with_types()
        labeled_pcd = segmentation.map_labels_to_original_points()
        if labeled_pcd is not None:
            o3d.visualization.draw_geometries([labeled_pcd], window_name="Labeled Point Cloud")
        #segmentation.extract_sections(n_main_stem=5, n_leaf=5)

        if 'leaf_angles' in modules_to_run:
            from photopack.point_cloud_analysis.point_cloud.leaf_angles import LeafAngleAnalyzer
            la = LeafAngleAnalyzer(segmentation)
            angles = la.compute_leaf_angles()
            logger.info(f"[leaf_angles] Computed leaf angles: {angles}")
            la.compute_leaf_angles()
            la.visualize_results()
            la.visualize_in_original()
 
        
       
        


        # Visualize the local regression results based on the anchor method.    
        #segmentation.visualize_results_anchor(num_main=5, num_leaf=5)

    # Compute volume ratios or other derived metrics
    def safe_div(a, b):
        return a / b if b else ""

    if "H_Max" in row_data and "V_100_unscaled" in row_data:
        row_data["H_over_V"] = safe_div(row_data["H_Max"], row_data["V_100_unscaled"])
    if "R_Max" in row_data and "V_100_unscaled" in row_data:
        row_data["R_over_V"] = safe_div(row_data["R_Max"], row_data["V_100_unscaled"])
    # etc. for other partial volumes if desired

    # Add scale and ring diameter
    row_data["Scale"] = scale
    row_data["Ring Diameter"] = ring_diameter

    # Log final row_data for debugging
    logger.info("[compute_metrics] Final row_data keys/values:")
    for k, v in row_data.items():
        logger.info(f"  {k} = {v}")

    return row_data

def process_single_file(ply_file, modules_to_run, seg_mode, scale, ring_diameter, top_percentage):
    """
    Perform the actual pipeline on one .ply file, returning a dict of metrics.
    """
    # Example cultivar naming => entire base name
    base = os.path.splitext(os.path.basename(ply_file))[0]
    cultivar = base

    # 1) Load the point cloud
    pcd = load_point_cloud(ply_file)

    # 2) Compute all metrics
    row_data = compute_metrics(pcd, modules_to_run, seg_mode, scale, ring_diameter, top_percentage)
    
    # 3) Insert the "Cultivar" to row_data
    row_data["Cultivar"] = cultivar

    return row_data

def main():
    parser = argparse.ArgumentParser(description='Full pipeline with JSON config, multi leaf angles, logging metrics.')
    parser.add_argument('path', help='A .ply file OR a directory of .ply files.')
    parser.add_argument('--diameter', type=float, default=1.3, help='Ring diameter in cm.')
    parser.add_argument('--module', choices=[
        'processing','convex_hull','hr_analysis','leaf_angles','projection','segmentation', 'mainstem_segmentation', 'all'
    ], nargs='+', help='Modules to run.')
    parser.add_argument('--segmentation-mode', choices=['auto','manual','both'],
                        default='auto', help='How to run segmentation.')
    parser.add_argument('--json-config', default='metrics_config.json', help='Path to JSON with base headers.')
    parser.add_argument('--csv-out', default='analysis_metrics.csv', help='Output CSV file.')
    parser.add_argument('--use_mysql', default=None, help='Output SQL file.')
    args = parser.parse_args()

    input_path = args.path
    ring_diameter = args.diameter
    modules_to_run = args.module
    seg_mode = args.segmentation_mode
    json_config = args.json_config
    csv_output = args.csv_out
    top_percentage = 60

    # If no modules specified or 'all', run them all
    available_modules = ['processing','convex_hull','hr_analysis','leaf_angles','projection','segmentation', 'mainstem_segmentation']
    if modules_to_run is None:
        logger.warning("No modules specified => using all modules.")
        modules_to_run = available_modules
    elif 'all' in modules_to_run:
        modules_to_run = available_modules

    # Compute scale
    TURNTABLE_DIAMETER = 39.878
    scale = TURNTABLE_DIAMETER / ring_diameter
    logger.info(f"Ring diameter: {ring_diameter}, Scale factor: {scale}")

    # 1) Load the base headers from JSON
    base_headers = load_metrics_config(json_config)

    # 2) Initialize the CSV
    init_csv(csv_output, base_headers)

    # 3) Single file or directory
    if os.path.isfile(input_path):
        if not input_path.lower().endswith('.ply'):
            logger.error(f"Input file is not .ply: {input_path}")
            sys.exit(1)
        ply_files = [input_path]
    elif os.path.isdir(input_path):
        ply_files = glob.glob(os.path.join(input_path, '*.ply'))
        if not ply_files:
            logger.warning(f"No .ply found in {input_path}")
    else:
        logger.error(f"Invalid path: {input_path}")
        sys.exit(1)

    # 4) Process each .ply
    for ply_file in ply_files:
        row_data = process_single_file(ply_file, modules_to_run, seg_mode, scale, ring_diameter, top_percentage)
        # Update CSV (overwriting if cultivar exists)
        append_to_csv(row_data)


    # Optionally, insert/update MySQL if desired.
    if args.use_mysql:  # Assume you add a --use-mysql flag
        from photopack.point_cloud_analysis.utils.mysql_database import insert_metrics_to_mysql
        db_config = {
            "user": args.mysql_user,
            "password": args.mysql_password,
            "host": args.mysql_host,
            "database": args.mysql_database
        }
        insert_metrics_to_mysql(row_data, db_config)










    logger.info("All processing completed.")

if __name__ == '__main__':
    main()
