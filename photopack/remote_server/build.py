#!/usr/bin/env python3
# build.py

import sys
import os
import time
import glob
import argparse
import logging
import subprocess
import shutil
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed


import cv2
import yaml
from tqdm import tqdm

def setup_logging(log_file: Path):
    """
    Sets up logging to output to both console and a log file.
    
    Args:
        log_file (Path): Path to the log file.
    """
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    
    # Console handler
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    
    # File handler
    fh = logging.FileHandler(log_file, mode='a')
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(formatter)
    
    # Avoid adding multiple handlers if already present
    if not logger.handlers:
        logger.addHandler(ch)
        logger.addHandler(fh)

def run_cmd(command: list, echo: bool = False) -> int:
    """
    Executes a shell command and streams the output in real-time.
    
    Args:
        command (list): The shell command and its arguments to execute.
        echo (bool): If True, logs the command before execution.
    
    Returns:
        int: The return code of the command.
    """
    if echo:
        logging.info(f"$ {' '.join(command)}")
    
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True
        )
        
        for line in process.stdout:
            print(line.strip())
        
        process.wait()
        if process.returncode != 0:
            logging.error(f"Command failed with return code {process.returncode}: {' '.join(command)}")
        return process.returncode
    except Exception as e:
        logging.error(f"Exception occurred while running command: {' '.join(command)}\nError: {e}")
        return -1

def process_single_image(image_path: Path, mask_folder: Path, a_cutoff: int, b_cutoff: int) -> None:
    """
    Processes a single image to create a mask.
    
    Args:
        image_path (Path): Path to the input image.
        mask_folder (Path): Directory to save the mask.
        a_cutoff (int): Cutoff value for the 'a' channel.
        b_cutoff (int): Cutoff value for the 'b' channel.
    """
    try:
        img = cv2.imread(str(image_path), 1)
        if img is None:
            logging.warning(f"Failed to read image: {image_path}")
            return
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

        a_channel = lab[:, :, 1]
        b_channel = lab[:, :, 2]

        # Masking
        _, b_mask = cv2.threshold(b_channel, b_cutoff, 128, cv2.THRESH_BINARY)
        _, a_mask = cv2.threshold(a_channel, a_cutoff, 127, cv2.THRESH_BINARY_INV)

        mask = a_mask + b_mask
        mask[mask < 129] = 0

        mask_filename = image_path.stem + '.png'
        mask_save_path = mask_folder / mask_filename
        cv2.imwrite(str(mask_save_path), mask)
        logging.info(f"Created mask for image: {mask_filename}")
    except cv2.error as e:
        logging.error(f"OpenCV error processing image {image_path}: {e}")
    except Exception as e:
        logging.error(f"Unexpected error processing image {image_path}: {e}")

def image_mask_func(image_dir: Path, mask_folder: Path, a_cutoff: int = 140, b_cutoff: int = 80, max_workers: int = None) -> None:
    """
    Applies a mask to all JPEG images in the specified directory and saves the masks to the mask folder.
    
    Args:
        image_dir (Path): Directory containing the original images.
        mask_folder (Path): Directory to save the masked images.
        a_cutoff (int): Cutoff value for the 'a' channel in LAB color space.
        b_cutoff (int): Cutoff value for the 'b' channel in LAB color space.
        max_workers (int, optional): Maximum number of threads to use. Defaults to None.
    """
    if not mask_folder.exists():
        mask_folder.mkdir(parents=True, exist_ok=True)
        logging.info(f"Created mask folder: {mask_folder}")
    
    images = list(image_dir.glob('*.jpg'))
    if not images:
        logging.warning(f"No JPEG images found in {image_dir}.")
        return
    
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [
            executor.submit(process_single_image, img_path, mask_folder, a_cutoff, b_cutoff)
            for img_path in images
        ]
        for _ in tqdm(as_completed(futures), total=len(futures), desc="Processing Images"):
            pass  # All logging is handled within process_single_image

def print_configuration(config: dict, output_file_path: Path) -> None:
    """
    Writes the current configuration to a specified file and prints it to the console.
    
    Args:
        config (dict): Configuration dictionary.
        output_file_path (Path): Path to the output configuration file.
    """
    try:
        with open(output_file_path, 'w') as f:
            f.write("Current Configuration:\n")
            for section, settings in config.items():
                f.write(f"[{section}]\n")
                for key, value in settings.items():
                    line = f"{key} = {value}"
                    print(line)
                    f.write(line + "\n")
                f.write("\n")
        logging.info(f"Configuration written to {output_file_path}")
    except IOError as e:
        logging.error(f"Failed to write configuration to {output_file_path}: {e}")






def run_colmap(work_space: dict, config: dict):
    """
    Executes a series of COLMAP commands to reconstruct the 3D model.

    Args:
        work_space (dict): Dictionary containing paths for COLMAP operations.
        config (dict): Configuration settings for COLMAP.
    """
    try:
        # Feature Extraction
        cmd_feature_extractor = [
            "colmap", "feature_extractor",
            "--database_path", str(work_space['DB_PATH']),
            "--image_path", str(work_space['IMAGE_PATH']),
            "--ImageReader.mask_path", str(work_space['MASK_PATH']),
            "--SiftExtraction.max_image_size", str(config['sift_extraction']['max_image_size']),
            "--SiftExtraction.domain_size_pooling", str(config['sift_extraction']['domain_size_pooling']).lower(),
            "--SiftExtraction.estimate_affine_shape", str(config['sift_extraction']['estimate_affine_shape']).lower(),
            "--SiftExtraction.max_num_features", str(config['sift_extraction']['max_num_features']),
            "--SiftExtraction.use_gpu", "1"  # Enable GPU acceleration
        ]

        # Exhaustive Matcher
        cmd_exhaustive_matcher = [
            "colmap", "exhaustive_matcher",
            "--database_path", str(work_space['DB_PATH']),
            "--ExhaustiveMatching.block_size", str(config['exhaustive_matcher'].get('block_size', 75)),
            "--SiftMatching.guided_matching", "1"  # Corrected option
        ]

        # Mapper
        cmd_mapper = [
            "colmap", "mapper",
            "--database_path", str(work_space['DB_PATH']),
            "--image_path", str(work_space['IMAGE_PATH']),
            "--output_path", str(work_space['SPARSE_PATH']),
            "--Mapper.ba_local_max_num_iterations", str(config['mapper']['ba_local_max_num_iterations']),
            "--Mapper.ba_global_max_num_iterations", str(config['mapper']['ba_global_max_num_iterations']),
            "--Mapper.ba_global_images_ratio", str(config['mapper']['ba_global_images_ratio']),
            "--Mapper.ba_global_points_ratio", str(config['mapper']['ba_global_points_ratio']),
            "--Mapper.ba_global_max_refinements", str(config['mapper']['ba_global_max_refinements']),
            "--Mapper.ba_local_max_refinements", str(config['mapper']['ba_local_max_refinements'])
        ]

        # Image Undistorter
        cmd_image_undistorter = [
            "colmap", "image_undistorter",
            "--image_path", str(work_space['IMAGE_PATH']),
            "--input_path", str(work_space['SPARSE_PATH'] / '0'),
            "--output_path", str(work_space['DENSE_PATH']),
            "--output_type", "COLMAP",
            "--max_image_size", str(config['patch_match_stereo']['max_image_size'])
        ]

        # Patch Match Stereo
        cmd_patch_match_stereo = [
            "colmap", "patch_match_stereo",
            "--workspace_path", str(work_space['DENSE_PATH']),
            "--workspace_format", "COLMAP",
            "--PatchMatchStereo.max_image_size", str(config['patch_match_stereo']['max_image_size']),
            "--PatchMatchStereo.window_radius", str(config['patch_match_stereo']['window_radius']),
            "--PatchMatchStereo.window_step", str(config['patch_match_stereo']['window_step']),
            "--PatchMatchStereo.num_samples", str(config['patch_match_stereo']['num_samples']),
            "--PatchMatchStereo.num_iterations", str(config['patch_match_stereo']['num_iterations']),
            "--PatchMatchStereo.write_consistency_graph", str(config['patch_match_stereo']['geom_consistency']).lower()
        ]

        # Stereo Fusion
        cmd_stereo_fusion = [
            "colmap", "stereo_fusion",
            "--workspace_path", str(work_space['DENSE_PATH']),
            "--workspace_format", "COLMAP",
            "--input_type", "geometric",
            "--output_path", str(work_space['DENSE_PATH'] / f"{work_space['OUTPUT_PATH']}_fused.ply"),
            "--StereoFusion.check_num_images", str(config['stereo_fusion']['check_num_images']),
            "--StereoFusion.max_image_size", str(config['stereo_fusion']['max_image_size'])
        ]

        # List of commands to run in sequence
        commands = [
            ("Feature Extraction", cmd_feature_extractor),
            ("Feature Matching", cmd_exhaustive_matcher),
            ("Sparse Reconstruction (Mapper)", cmd_mapper),
            ("Image Undistorter", cmd_image_undistorter),
            ("Patch Match Stereo", cmd_patch_match_stereo),
            ("Stereo Fusion", cmd_stereo_fusion)
        ]

        for step_name, cmd in commands:
            logging.info(f"Starting step: {step_name}")
            logging.debug(f"Executing command: {' '.join(cmd)}")  # Detailed command log
            rc = run_cmd(cmd, echo=True)
            if rc != 0:
                logging.error(f"COLMAP command failed at step: {step_name}")
                sys.exit(rc)
            else:
                logging.info(f"Completed step: {step_name}")

        logging.info("COLMAP reconstruction completed successfully.")
    
    except Exception as e:
        logging.error(f"An error occurred during COLMAP execution: {e}")
        sys.exit(1)





def default_configuration() -> dict:
    """
    Returns the default configuration settings.
    
    Returns:
        dict: Default configuration dictionary.
    """
    return {
        'sift_extraction': {
            'max_image_size': 3200,
            'max_num_features': 8192,
            'estimate_affine_shape': True,
            'domain_size_pooling': True,
        },
        'exhaustive_matcher': {
            'guided_matching': True,
            'block_size': 75,  # Added default block_size
            'loop_detection_num_images': 30,  # Added default loop_detection_num_images
        },
        'mapper': {
            'ba_local_max_num_iterations': 30,
            'ba_global_max_num_iterations': 75,
            'ba_global_images_ratio': 1.1,
            'ba_global_points_ratio': 1.1,
            'ba_global_max_refinements': 5,
            'ba_local_max_refinements': 2,
        },
        'patch_match_stereo': {
            'max_image_size': 3200,
            'window_radius': 5,
            'window_step': 1,
            'num_samples': 15,
            'num_iterations': 5,
            'geom_consistency': False,
        },
        'stereo_fusion': {
            'check_num_images': 50,
            'max_image_size': 3200,
        },
    }

def medium_quality(config: dict) -> dict:
    """
    Adjusts the configuration for medium quality processing.
    
    Args:
        config (dict): Original configuration dictionary.
    
    Returns:
        dict: Modified configuration dictionary for medium quality.
    """
    config = config.copy()
    config['sift_extraction']['max_image_size'] = 1600
    config['sift_extraction']['max_num_features'] = 4096
    config['exhaustive_matcher']['loop_detection_num_images'] = int(config['exhaustive_matcher']['loop_detection_num_images'] / 1.5)
    #config['vocab_tree_matching']['num_images'] = int(config['vocab_tree_matching']['num_images'] / 1.5)
    config['mapper']['ba_local_max_num_iterations'] = int(config['mapper']['ba_local_max_num_iterations'] / 1.5)
    config['mapper']['ba_global_max_num_iterations'] = int(config['mapper']['ba_global_max_num_iterations'] / 1.5)
    config['mapper']['ba_global_images_ratio'] *= 1.1
    config['mapper']['ba_global_points_ratio'] *= 1.1
    config['mapper']['ba_global_max_refinements'] = 2
    config['patch_match_stereo']['max_image_size'] = 1600
    config['patch_match_stereo']['window_radius'] = 4
    config['patch_match_stereo']['window_step'] = 2
    config['patch_match_stereo']['num_samples'] = int(config['patch_match_stereo']['num_samples'] / 1.5)
    config['patch_match_stereo']['num_iterations'] = 5
    config['patch_match_stereo']['geom_consistency'] = False
    config['stereo_fusion']['check_num_images'] = int(config['stereo_fusion']['check_num_images'] / 1.5)
    config['stereo_fusion']['max_image_size'] = 1600
    return config

def high_quality(config: dict) -> dict:
    """
    Adjusts the configuration for high quality processing.
    
    Args:
        config (dict): Original configuration dictionary.
    
    Returns:
        dict: Modified configuration dictionary for high quality.
    """
    config = config.copy()
    config['sift_extraction']['estimate_affine_shape'] = True
    config['sift_extraction']['max_image_size'] = 2400
    config['sift_extraction']['max_num_features'] = 8192
    config['exhaustive_matcher']['guided_matching'] = True
    config['exhaustive_matcher']['block_size'] = 75
    config['mapper']['ba_local_max_num_iterations'] = 30
    config['mapper']['ba_local_max_refinements'] = 3
    config['mapper']['ba_global_max_num_iterations'] = 75
    config['patch_match_stereo']['max_image_size'] = 2400
    config['stereo_fusion']['max_image_size'] = 2400
    return config

def extreme_quality(config: dict) -> dict:
    """
    Adjusts the configuration for extreme quality processing.
    
    Args:
        config (dict): Original configuration dictionary.
    
    Returns:
        dict: Modified configuration dictionary for extreme quality.
    """
    config = config.copy()
    config['sift_extraction']['estimate_affine_shape'] = True
    config['sift_extraction']['domain_size_pooling'] = True
    config['exhaustive_matcher']['guided_matching'] = True
    config['mapper']['ba_local_max_num_iterations'] = 40
    config['mapper']['ba_local_max_refinements'] = 3
    config['mapper']['ba_global_max_num_iterations'] = 100
    return config


def load_configuration(config_file: Path) -> dict:
    """
    Loads configuration settings from a YAML file.
    
    Args:
        config_file (Path): Path to the YAML configuration file.
    
    Returns:
        dict: Configuration dictionary.
    """
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        logging.info(f"Loaded configuration from {config_file}")
        return config
    except FileNotFoundError:
        logging.error(f"Configuration file not found: {config_file}")
        sys.exit(1)
    except yaml.YAMLError as e:
        logging.error(f"Error parsing YAML file {config_file}: {e}")
        sys.exit(1)

def validate_configuration(config: dict) -> None:
    """
    Validates the configuration parameters.
    
    Args:
        config (dict): Configuration dictionary.
    
    Raises:
        ValueError: If any configuration parameter is invalid.
    """
    if not isinstance(config['sift_extraction']['max_image_size'], int) or config['sift_extraction']['max_image_size'] <= 0:
        raise ValueError("sift_extraction.max_image_size must be a positive integer.")
    
    if not isinstance(config['sift_extraction']['max_num_features'], int) or config['sift_extraction']['max_num_features'] <= 0:
        raise ValueError("sift_extraction.max_num_features must be a positive integer.")
    
    # Add more validation rules as necessary

def check_colmap_installed() -> None:
    """
    Checks if COLMAP is installed and accessible.
    """
    if shutil.which("colmap") is None:
        logging.error("COLMAP is not installed or not found in PATH.")
        sys.exit(1)
    else:
        logging.info("COLMAP is installed and found in PATH.")


def main():
    """
    Main function to execute the build process.
    """
    # Argument parsing
    parser = argparse.ArgumentParser(description="Reconstruct a 3D model from images using COLMAP.")
    parser.add_argument('quality', choices=['low', 'medium', 'high', 'extreme'], help="Quality level for reconstruction.")
    parser.add_argument('workspace', type=Path, help="Path to the workspace directory.")
    parser.add_argument('--config', type=Path, default=Path('config.yaml'), help="Path to the configuration YAML file.")
    args = parser.parse_args()

    quality = args.quality.lower()
    workspace = args.workspace.resolve()

    # Setup logging
    log_file = workspace / "build.log"
    setup_logging(log_file)

    logging.info(f"Starting build process with quality: {quality}")
    logging.info(f"Workspace directory: {workspace}")

    # Check COLMAP installation
    #check_colmap_installed()

    # Load configuration
    config = load_configuration(args.config)

    # Validate configuration
    try:
        validate_configuration(config)
    except ValueError as e:
        logging.error(f"Configuration validation error: {e}")
        sys.exit(1)

    # Adjust configuration based on quality
    if quality == "medium":
        config = medium_quality(config)
        logging.info("Applied medium quality configuration.")
    elif quality == "high":
        config = high_quality(config)
        logging.info("Applied high quality configuration.")
    elif quality == "extreme":
        config = extreme_quality(config)
        logging.info("Applied extreme quality configuration.")
    else:
        logging.info("Using default configuration.")

    # Define workspace paths using pathlib
    work_space = {
        'DB_PATH': workspace / "database.db",
        'IMAGE_PATH': workspace / "images",
        'SPARSE_PATH': workspace / "sparse",
        'DENSE_PATH': workspace / "dense",
        'MASK_PATH': workspace / "mask_images",
        'WORK_FILE': workspace,
        'OUTPUT_PATH': workspace.name
    }

    # Create necessary directories
    for path in [work_space['SPARSE_PATH'], work_space['DENSE_PATH'], work_space['MASK_PATH']]:
        path.mkdir(parents=True, exist_ok=True)
        logging.info(f"Ensured directory exists: {path}")

    # Check if image directory exists and contains images
    if not work_space['IMAGE_PATH'].exists():
        logging.error(f"Image directory does not exist: {work_space['IMAGE_PATH']}")
        sys.exit(1)
    
    if not list(work_space['IMAGE_PATH'].glob('*.jpg')):
        logging.error(f"No JPEG images found in: {work_space['IMAGE_PATH']}")
        sys.exit(1)

    # Process images with progress bar
    image_mask_func(work_space['IMAGE_PATH'], work_space['MASK_PATH'])

    # Save configuration
    configuration_file = workspace / "configuration.txt"
    print_configuration(config, configuration_file)

    # Run COLMAP
    start_time = time.time()
    #run_colmap(work_space, config)
    elapsed_time = time.time() - start_time
    logging.info(f"COLMAP reconstruction completed in {elapsed_time:.2f} seconds.")

if __name__ == "__main__":
    main()