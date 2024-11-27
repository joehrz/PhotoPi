# PhotoPi/photopack/point_cloud_analysis/main.py

import argparse
import open3d as o3d
import numpy as np
import logging
import sys
import os

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

def load_point_cloud(filename):
    """
    Loads a point cloud from a file.

    Args:
        filename (str): Path to the point cloud file (e.g., .ply).

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

def main():
    # Define TURNTABLE_DIAMETER here
    TURNTABLE_DIAMETER = 39.878  # Diameter of the turntable in centimeters

    parser = argparse.ArgumentParser(description='Point Cloud Analysis Tool')
    parser.add_argument('filename', help='Input point cloud file (e.g., .ply)')
    parser.add_argument('--diameter', type=float, default=1.3, help='Ring diameter of the turntable in centimeters')
    parser.add_argument('--module', choices=['processing','convex_hull', 'hr_analysis', 'leaf_angles', 'projection', 'all'], nargs='+',
                        help='Modules to run. Choices: convex_hull, hr_analysis, projection, all. You can specify multiple modules separated by space.')
    args = parser.parse_args()
    filename = args.filename
    ring_diameter = args.diameter
    modules_to_run = args.module



    # Compute scale factor based on the turntable diameter
    scale = TURNTABLE_DIAMETER / ring_diameter
    top_percentage = 60

    logger.info(f"Ring Diameter: {ring_diameter} cm")
    logger.info(f"Scale Factor: {scale}")

    logger.info(f"Processing file: {filename}")

    # Load the point cloud
    point_cloud = load_point_cloud(filename)

    if point_cloud is not None:

        # Define available modules
        available_modules = ['processing,''convex_hull', 'hr_analysis','leaf_angles', 'projection']

        # Check if modules are specified; if not, set default modules
        if modules_to_run is None:
            logger.warning("No modules specified. Running all modules by default.")
            modules_to_run = available_modules
        elif 'all' in modules_to_run:
            modules_to_run = available_modules


        if 'processing' in modules_to_run:
            logger.info("Running Point Cloud Processing module...")
            try:
                from photopack.point_cloud_analysis.point_cloud.processing import PointCloudProcessor
            except ModuleNotFoundError as e:
                logger.error(f"Failed to import processing module: {e}")
                sys.exit(1)



            work_dir = filename.rsplit('\\', 1)[-1]
            main_dir = filename.rsplit('\\', 1)[0]

            out = work_dir.rsplit('.', 1)[0]


            output_path = os.path.join(main_dir, out +'_output.ply')
            # Initialize the PointCloudProcessor
            processor = PointCloudProcessor(point_cloud)

            # Process the point cloud
            processor.process()

            # Save the processed point cloud
            #output_path = 'processed_point_cloud.ply'
            processor.save_processed_point_cloud(output_path)
            processor.visualize()


        # Import and run modules based on command-line arguments
        if 'convex_hull' in modules_to_run:
            logger.info("Running Convex Hull module...")
            try:
                from photopack.point_cloud_analysis.point_cloud.convex_hull import ConvexHullAnalyzer
            except ModuleNotFoundError as e:
                logger.error(f"Failed to import convex_hull module: {e}")
                sys.exit(1)

            # Initialize the ConvexHullAnalyzer
            convex_hull_analyzer = ConvexHullAnalyzer(point_cloud)

            # Compute Convex Hull Volume
            convex_hull_volume = convex_hull_analyzer.compute_convex_hull_volume(scale=scale)
            logger.info(f"Convex Hull Volume: {convex_hull_volume:.2f} cm³")

            # Visualize Convex Hull (optional)
            convex_hull_analyzer.visualize_convex_hull()
            
            percentile_volume = convex_hull_analyzer.process_pc_percentile_volume(top_percentage, scale)
            logger.info(f"Top {top_percentage}% Convex Hull Volume: {percentile_volume:.2f} cm³")

        if 'hr_analysis' in modules_to_run:
            logger.info("Running H/R Analysis module...")
            try:
                from photopack.point_cloud_analysis.point_cloud.hr_analysis import HRAnalyzer
            except ModuleNotFoundError as e:
                logger.error(f"Failed to import hr_analysis module: {e}")
                sys.exit(1)

            # Initialize the HRAnalyzer
            hr_analyzer = HRAnalyzer(point_cloud, scale)

            # Perform the analysis
            hr_analyzer.analyze_hr()



            hr_analyzer.plot_density_vs_z()
            hr_analyzer.plot_height_vs_radius()
            hr_analyzer.plot_mean_r_vs_z()

        if 'leaf_angles' in modules_to_run:
            logger.info("Running Leaf Angles module...")
            try:
                from photopack.point_cloud_analysis.point_cloud.leaf_angles import LeafAngleAnalyzer
            except ModuleNotFoundError as e:
                logger.error(f"Failed to import leaf_angles module: {e}")
                sys.exit(1)

            # Initialize the LeafAngleAnalyzer with the point cloud
            analyzer = LeafAngleAnalyzer(point_cloud)

            # Run the analysis steps
            analyzer.process_point_cloud(eps=0.006, min_samples=50, n_sections=80)
            analyzer.aggregate_centroids()
            analyzer.build_graph(k=2)
            analyzer.segment_nodes()
            analyzer.refine_segmentation()
            analyzer.remove_cycles()
            analyzer.label_branch_off_nodes()
            analyzer.refine_classification(iterations=50)
            analyzer.reclassify_stem_nodes()
            analyzer.extract_sections(n_main_stem=5, n_leaf=5)
            analyzer.compute_leaf_angles()

            # Optionally, visualize the results
            analyzer.visualize_results()

            # Access the computed leaf angles
            leaf_angles = analyzer.angles
            logger.info(f"Leaf angles: {leaf_angles}")

        if 'projection' in modules_to_run:
            logger.info("Running Projection module...")
            try:
                from photopack.point_cloud_analysis.point_cloud.projection import ProjectionAnalyzer
            except ModuleNotFoundError as e:
                logger.error(f"Failed to import projection module: {e}")
                sys.exit(1)

            # Initialize the ProjectionAnalyzer
            projection_analyzer = ProjectionAnalyzer(point_cloud)

            # Compute Alpha Shape Area
            alpha_area = projection_analyzer.compute_alpha_shape_area(alpha=1.0)
            logger.info(f"Alpha shape area: {alpha_area:.4f} square units")

            # Optionally, plot results
            # projection_analyzer.plot_original_points(save_fig=True)
            # projection_analyzer.plot_alpha_shape(save_fig=True)
            # projection_analyzer.plot_points_and_alpha_shape(save_fig=True)

if __name__ == '__main__':
    main()
