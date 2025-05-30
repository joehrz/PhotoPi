# PhotoPi: Remote Server (3D Reconstruction)

This directory provides the scripts and configuration for the remote server component of the PhotoPi project. The primary function of this component is to perform 3D reconstruction from sets of 2D images using COLMAP, a general-purpose Structure-from-Motion (SfM) and Multi-View Stereo (MVS) pipeline.

For a general overview of the entire PhotoPi project, please see the [main README.md](../../../README.md).

## Table of Contents

* [Overview](#overview)
* [Core Components](#core-components)
* [System Prerequisites](#system-prerequisites)
* [Setup and Deployment](#setup-and-deployment)
* [Input Data Structure](#input-data-structure)
* [Configuration (`config.yaml`)](#configuration-configyaml)
* [Running the Reconstruction (`build.py`)](#running-the-reconstruction-buildpy)
    * [Command-Line Arguments](#command-line-arguments)
    * [Quality Presets](#quality-presets)
* [Output Structure](#output-structure)
* [Troubleshooting](#troubleshooting)

## Overview

The remote server handles the computationally demanding task of 3D model generation. By offloading this process, the Raspberry Pi and the user's main computer are freed up. The `build.py` script automates the various stages of the COLMAP pipeline, from feature extraction to dense point cloud generation.

## Core Components

* **`build.py`**: The main Python script that orchestrates the COLMAP pipeline. Its responsibilities include:
    * Setting up the COLMAP workspace.
    * Creating image masks to exclude background regions using LAB color space thresholding via `image_mask_func`.
    * Executing COLMAP commands for:
        * Feature extraction (`colmap feature_extractor`).
        * Feature matching (`colmap exhaustive_matcher`).
        * Sparse reconstruction / Structure-from-Motion (`colmap mapper`).
        * Image undistortion (`colmap image_undistorter`).
        * Dense reconstruction / Multi-View Stereo (`colmap patch_match_stereo`).
        * Point cloud fusion (`colmap stereo_fusion`).
    * Loading and applying parameters from `config.yaml`.
    * Implementing quality presets (`low`, `medium`, `high`, `extreme`) that adjust COLMAP parameters.
    * Logging the process to `build.log` and printing configuration to `configuration.txt`.
* **`config.yaml`**: A YAML configuration file that specifies detailed parameters for each step of the COLMAP pipeline. This allows for fine-grained control over the reconstruction process.
* **Deployment Script**: The `deploy_to_remote_server.sh` script (located in the project root) is used to deploy these files and set up the environment on the remote server.

## System Prerequisites

* A server (Linux recommended) with:
    * Sufficient CPU resources.
    * Ample RAM (COLMAP can be memory-intensive).
    * A CUDA-enabled NVIDIA GPU is highly recommended for significant speed-ups. The `build.py` script attempts to use the GPU for feature extraction (`--SiftExtraction.use_gpu 1`).
    * Sufficient disk space.
* **COLMAP**: Must be installed system-wide, and the `colmap` executable must be in the system's PATH. The `build.py` script checks for this using `shutil.which("colmap")`.
* **Python 3.x**.
* **Python Packages** (typically installed by `deploy_to_remote_server.sh` into a virtual environment based on `remote_server_deps` in `setup.py`):
    * `opencv-python` (for image masking)
    * `pyyaml` (for loading `config.yaml`)
    * `tqdm` (for progress bars, used in `image_mask_func`)

## Setup and Deployment

The preferred method for deploying this component is via the `deploy_to_remote_server.sh` script from your local machine (PhotoPi project root).

```bash
# From your Control PC, in the PhotoPi project root directory:
# Ensure your .env file in the project root has REMOTE_USER, REMOTE_HOST, REMOTE_PORT, REMOTE_SSH_KEY defined.
./deploy_to_remote_server.sh