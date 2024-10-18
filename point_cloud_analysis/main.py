# PhotoPi/point_cloud_analysis/main.py

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
    args = parser.parse_args()
    filename = args.filename
    ring_diameter = args.diameter

    # Compute scale factor based on the turntable diameter
    scale = TURNTABLE_DIAMETER / ring_diameter

    logger.info(f"Ring Diameter: {ring_diameter} cm")
    logger.info(f"Scale Factor: {scale}")

    logger.info(f"Processing file: {filename}")

    # Load the point cloud
    point_cloud = load_point_cloud(filename)


    if point_cloud is not None:

        # Import functions from submodules using package-based imports
        try:
            from point_cloud.convex_hull import (
                compute_convex_hull_volume,
                visualize_convex_hull,
            )
            from point_cloud.hr_analysis import analyze_hr
        except ModuleNotFoundError as e:
            logger.error(f"Failed to import modules: {e}")
            sys.exit(1)

        # Compute Convex Hull Volume using SciPy
        points = np.asarray(point_cloud.points)
        convex_hull_volume = compute_convex_hull_volume(points, scale=scale)
        logger.info(f"Convex Hull Volume: {convex_hull_volume:.2f}")

        # Visualize Convex Hull using Open3D
        visualize_convex_hull(point_cloud, color=(1, 0, 0))

        # Perform H/R Analysis
        analysis_results = analyze_hr(point_cloud, scale)
        logger.info(f"Analysis Results: {analysis_results}")

if __name__ == '__main__':
    main()

