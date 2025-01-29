# PhotoPi/photopack/point_cloud_analysis/main.py

import argparse
import open3d as o3d
import numpy as np
import logging
import sys
import os
import glob

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


def load_point_cloud(filename):
    """
    Loads a point cloud from a file (.ply).

    Args:
        filename (str): Path to the point cloud file.
    Returns:
        o3d.geometry.PointCloud: The loaded point cloud.
    """
    try:
        pcd = o3d.io.read_point_cloud(filename)
        if not pcd.has_points():
            logger.error(f"No points found in the point cloud file: {filename}")
            sys.exit(1)
        logger.info(f"Successfully loaded point cloud from {filename}")
        return pcd
    except FileNotFoundError:
        logger.error(f"File not found: {filename}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Unexpected error loading point cloud: {e}")
        sys.exit(1)


def process_single_file(ply_file, modules_to_run, seg_mode, scale, top_percentage):
    """
    Process a single .ply file using the selected modules, including optional segmentation steps.
    Saves final segmentation results to 'processed_combined' (if segmentation is run).
    """
    # 1) Load the point cloud
    logger.info(f"Processing file: {ply_file}")
    point_cloud = load_point_cloud(ply_file)

    # For reference, let's define these directories:
    raw_dir = os.path.dirname(ply_file)                 # e.g.  point_cloud_data/raw
    point_cloud_data_dir = os.path.dirname(raw_dir)     # e.g.  point_cloud_data
    processed_combined_dir = os.path.join(point_cloud_data_dir, "processed_combined")

    # Ensure output directory exists
    os.makedirs(processed_combined_dir, exist_ok=True)

    # We'll extract a base name like "mycloud_fused_output"
    base = os.path.splitext(os.path.basename(ply_file))[0]

    # 1) PROCESSING
    if 'processing' in modules_to_run:
        logger.info("Running Point Cloud Processing module...")
        try:
            from photopack.point_cloud_analysis.point_cloud.processing import PointCloudProcessor
        except ModuleNotFoundError as e:
            logger.error(f"Failed to import processing module: {e}")
            sys.exit(1)

        processor = PointCloudProcessor(point_cloud)
        processor.process()

        # Overwrite our point_cloud with the processed version if the module does so
        point_cloud = processor.get_processed_point_cloud()
        # Optionally save or visualize
        # processor.save_processed_point_cloud(...)
        # processor.visualize()

    # 2) CONVEX HULL
    if 'convex_hull' in modules_to_run:
        logger.info("Running Convex Hull module...")
        try:
            from photopack.point_cloud_analysis.point_cloud.convex_hull import ConvexHullAnalyzer
        except ModuleNotFoundError as e:
            logger.error(f"Failed to import convex_hull module: {e}")
            sys.exit(1)

        hull_analyzer = ConvexHullAnalyzer(point_cloud)
        cvol = hull_analyzer.compute_convex_hull_volume(scale=scale)
        logger.info(f"Convex Hull Volume: {cvol:.2f} cm³")

        pvol = hull_analyzer.process_pc_percentile_volume(top_percentage, scale)
        logger.info(f"Top {top_percentage}% Convex Hull Volume: {pvol:.2f} cm³")
        # If storing, do so in a dict or DB, etc.

        # Optional visualization
        # hull_analyzer.visualize_convex_hull()

    # 3) H/R ANALYSIS
    if 'hr_analysis' in modules_to_run:
        logger.info("Running H/R Analysis module...")
        try:
            from photopack.point_cloud_analysis.point_cloud.hr_analysis import HRAnalyzer
        except ModuleNotFoundError as e:
            logger.error(f"Failed to import hr_analysis module: {e}")
            sys.exit(1)

        hr_analyzer = HRAnalyzer(point_cloud, scale)
        hr_analyzer.analyze_hr()
        hr_analyzer.plot_density_vs_z()
        hr_analyzer.plot_height_vs_radius()
        hr_analyzer.plot_mean_r_vs_z()

    # 4) LEAF ANGLES
    if 'leaf_angles' in modules_to_run:
        logger.info("Running Leaf Angles module...")
        try:
            from photopack.point_cloud_analysis.point_cloud.leaf_angles import LeafAngleAnalyzer
        except ModuleNotFoundError as e:
            logger.error(f"Failed to import leaf_angles module: {e}")
            sys.exit(1)

        la = LeafAngleAnalyzer(point_cloud)
        la.process_point_cloud(eps=0.006, min_samples=50, n_sections=80)
        la.aggregate_centroids()
        la.build_graph(k=2)
        la.segment_nodes()
        la.refine_segmentation()
        la.remove_cycles()
        la.label_branch_off_nodes()
        la.refine_classification(iterations=50)
        la.reclassify_stem_nodes()
        la.extract_sections(n_main_stem=5, n_leaf=5)
        la.compute_leaf_angles()
        la.visualize_results()

        logger.info(f"Leaf angles: {la.angles}")

    # 5) PROJECTION
    if 'projection' in modules_to_run:
        logger.info("Running Projection module...")
        try:
            from photopack.point_cloud_analysis.point_cloud.projection import ProjectionAnalyzer
        except ModuleNotFoundError as e:
            logger.error(f"Failed to import projection module: {e}")
            sys.exit(1)

        proj_analyzer = ProjectionAnalyzer(point_cloud)
        alpha_area = proj_analyzer.compute_alpha_shape_area(alpha=1.0)
        logger.info(f"Alpha shape area: {alpha_area:.4f} square units")
        # Optionally store or plot

    # 6) SEGMENTATION
    if 'segmentation' in modules_to_run:
        logger.info("Running Segmentation module...")
        try:
            from photopack.point_cloud_analysis.point_cloud.segmentation_extractor import Segmentation
        except ModuleNotFoundError as e:
            logger.error(f"Failed to import segmentation_extractor (Segmentation) module: {e}")
            sys.exit(1)

        # For the "auto" or "both" modes
        if seg_mode in ['auto', 'both']:
            logger.info("Performing AUTOMATIC segmentation steps...")
            seg = Segmentation(point_cloud, min_cluster_size=50)

            seg.preprocess(voxel_size=0.005, nb_neighbors=20, std_ratio=2.0)
            seg.graph_based_segmentation(k=50, z_scale=1.0, distance_threshold=0.009)
            seg.refine_with_mst_segmentation(num_cuts=2)
            seg.refine_clusters(min_cluster_size=30)

            # Possibly pick top 3 largest clusters for extra MST pass
            cluster_sizes = seg.get_cluster_sizes_downsampled()
            filtered_clusters = [(lbl, sz) for lbl, sz in cluster_sizes.items() if lbl != -1]
            sorted_clusters = sorted(filtered_clusters, key=lambda x: x[1], reverse=True)
            topN = 3
            clusters_to_refine = [lbl for (lbl, _) in sorted_clusters[:topN]]
            logger.info(f"Clusters selected for further refinement: {clusters_to_refine}")

            if clusters_to_refine:
                seg.refine_with_mst_segmentation(num_cuts=1, clusters_to_refine=clusters_to_refine)
                seg.refine_clusters(min_cluster_size=30)

            seg.map_labels_to_original()
            seg.segment_and_store()
            seg.visualize_segments_original()

            # Now save final .npz and .ply to processed_combined folder
            npz_output = os.path.join(processed_combined_dir, f"{base}_segmented.npz")
            seg.save_numpy_arrays(npz_output)

            seg.save_combined_segmented_clouds(processed_combined_dir, base)
            logger.info("Automatic segmentation step completed!")

        if seg_mode in ['manual', 'both']:
            logger.info("Performing MANUAL segmentation adjustments...")

            from photopack.point_cloud_analysis.segmentation_extractor import Segmentation
            seg_manual = Segmentation(point_cloud)
            # Load existing .npz from processed_combined
            npz_path = os.path.join(processed_combined_dir, f"{base}_segmented.npz")

            if os.path.exists(npz_path):
                seg_manual.load_numpy_arrays(npz_path)
                logger.info(f"Loaded existing segmentation data from {npz_path}")
            else:
                logger.warning(f"No auto-seg data found at {npz_path}. Proceeding without it...")

            # Suppose your manual-labeled .plys go in: point_cloud_data/manual_labels
            manual_labels_dir = os.path.join(point_cloud_data_dir, "manual_labels")
            if not os.path.exists(manual_labels_dir):
                logger.warning(f"Manual labels directory does not exist: {manual_labels_dir}")

            manual_files = glob.glob(os.path.join(manual_labels_dir, f"{base}_cluster_*.ply"))
            logger.info(f"Found {len(manual_files)} manual-labeled files: {manual_files}")

            seg_manual.update_labels_from_manual_segmentation(
                manual_files,
                base_filename=base,
                outdir=processed_combined_dir,
                min_points=2000
            )
            logger.info("Manual segmentation merge completed!")


def main():
    # Basic config
    TURNTABLE_DIAMETER = 39.878  # cm
    parser = argparse.ArgumentParser(description='Point Cloud Analysis Tool')
    parser.add_argument('path', help='A .ply file OR a directory of .ply files.')
    parser.add_argument('--diameter', type=float, default=1.3, help='Ring diameter in cm (for scaling).')
    parser.add_argument('--module', choices=[
        'processing','convex_hull','hr_analysis','leaf_angles','projection','segmentation','all'
    ], nargs='+', help='Modules to run. E.g. "convex_hull" or "all".')
    parser.add_argument('--segmentation-mode', choices=['auto','manual','both'],
                        default='auto',
                        help='Run segmentation in "auto", "manual", or "both" modes.')
    args = parser.parse_args()

    input_path = args.path
    ring_diameter = args.diameter
    modules_to_run = args.module
    seg_mode = args.segmentation_mode  # 'auto', 'manual', 'both'
    scale = TURNTABLE_DIAMETER / ring_diameter
    top_percentage = 60

    logger.info(f"Ring Diameter: {ring_diameter} cm")
    logger.info(f"Scale Factor: {scale}")

    # Figure out if user passed a single file or a directory
    if os.path.isfile(input_path):
        if not input_path.lower().endswith(".ply"):
            logger.error(f"Input file is not a .ply: {input_path}")
            sys.exit(1)
        ply_files = [input_path]
    elif os.path.isdir(input_path):
        ply_files = glob.glob(os.path.join(input_path, "*.ply"))
        if not ply_files:
            logger.warning(f"No .ply files found in directory: {input_path}")
    else:
        logger.error(f"Input path is neither a file nor directory: {input_path}")
        sys.exit(1)

    # If no modules specified or 'all', run all
    available_modules = ['processing','convex_hull','hr_analysis','leaf_angles','projection','segmentation']
    if modules_to_run is None:
        logger.warning("No modules specified. Running all modules by default.")
        modules_to_run = available_modules
    elif 'all' in modules_to_run:
        modules_to_run = available_modules

    # Process each .ply in the list
    for ply_file in ply_files:
        process_single_file(ply_file, modules_to_run, seg_mode, scale, top_percentage)

    logger.info("All processing completed.")


if __name__ == '__main__':
    main()

