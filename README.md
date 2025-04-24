# PhotoPi

**PhotoPi** is a Python-based GUI application designed for 3D plant phenotyping using photogrammetry. It automates the capture of 2D images to construct 3D models of plants. The system controls multiple cameras and a turntable via a Raspberry Pi, ensuring precise management of the imaging process. Operated remotely from a main computer, PhotoPi integrates into photogrammetry workflows and offers robust tools for point cloud analysis.


## Features

- **Dynamic SSH Connection Management:** Connect and disconnect from the Raspberry Pi systems over SSH.
- **User Authentication and Network Configuration:** Simple GUI-based authentication with support for configuring network settings.
- **Live Camera Control and Image Capture:** Directly control multiple cameras, capture images, and manage storage locations.
- **Configurable Settings:** Customize camera angles, image capture timings, and other settings to fit various use cases.
- **Inspection and Management:** View and manage captured images directly through the GUI.
- **Reset Function:** Quickly reset the system in case of failures.
- **Remote Photogrammetry Processing:** Send captured images to a remote server to perform Structure from Motion (SFM) photogrammetry and reconstruct 3D models.
- **Point Cloud Analysis Tools:** Analyze 3D models of plants using point cloud processing techniques, including convex hull computation, height/radius analysis, leaf angle measurement.

## Architecture

PhotoPi consists of several interconnected components:

1. **Main System:** Runs the GUI application on a primary computer, managing SSH connections, camera settings, and image transfers.
2. **Raspberry Pi:** Controls the cameras and turntable, handles image capture, and stores images locally.
3. **Remote Server:** Facilitates deployment, management, and integration of PhotoPi into larger photogrammetry workflows.
4. **Point Cloud Analysis:** Provides tools for processing and analyzing 3D models generated from captured images.

## Hardware

The following table provides a breakdown of the main components and associated costs for the photogrammetry rig setup:

| **Part** | **Brand** | **Model** | **Price** | **Specs** | **Description** |
|----------|-----------|-----------|-----------|-----------|-----------------|
| Embedded system | Raspberry Pi | Raspberry Pi 4 Model B | $249.99 | Quad-core Cortex-A72, 4GB RAM, 40 pin GPIO header, 2-lane MIPI CSI camera port | Compact, low-cost computer board ideal for embedded applications |
| 4 RGB Cameras | Arducam | 64MP Autofocus Quad-Camera Kit | $199 | WiFi/Bluetooth, Res: 4056 x 3040 px, FOV diagonal: 84° | High-resolution, auto-focus RGB cameras for detailed image capture |
| Featureless Background | Rose Brand | 62" Poly Pro | $15.20 /yard | Fabric, Chroma Key Blue | Wrinkle-resistant fabric providing a uniform backdrop for imaging |
| Stepper Motor | Adafruit | Stepper Motor HAT - mini kit | $27.95 | TB6612 chipset, 4.5VDC to 13.5VDC, unipolar or bipolar | Motor HAT (Hardware Attached on Top) designed for precise motion control with the Raspberry Pi |
| Turntable | Ortery | PhotoCapture 360M | $1,200 | 28.956 and 39.878 cm diameter platforms | Software-controlled rotary table for consistent image capture angles |
| Aluminum Extrusion | Misumi | | $950 | Aluminum alloy, Square shape, 40mm main frame and arms size, four side slots | Frame with stationary mounts, angle brackets, nuts and screws |
| All-Purpose LED Grow Lights | Root Farm | | $145 each | 45W per panel, Grow area: 25 cm (10 in) x 55 cm (21.5 in) per panel, Adjustable legs with 360° swivel | Two growing light panels consisting of 3-band LEDs |

## Wiring Diagram

<p><img src="docs/images/pi_stepp_schem.svg" alt="Schematic of the modified wiring required to drive the the turntable stepper motor via the RPi and Adafruit DC & Stepper Motor HAT."></p>
<p><em>Schematic of the modified wiring required to drive the the turntable stepper motor.</em></p>


## Requirements

### Main System

- **Operating System:** Windows, macOS, or Linux
- **Python:** 3.x

- **Python Packages:**
  - `pillow==10.4.0`
  - `paramiko==3.1.0`

### Raspberry Pi

- **Hardware:**
  - Raspberry Pi (recommended: Pi 4)
  - 64MP Autofocus Synchronized Quad-Camera Kit for Raspberry Pi
  - Turntable
- **Software:**
  - Raspberry Pi OS (latest version)
  - Python 3.x

- **Python Packages:**
  - `RPi.GPIO`
  - `adafruit-circuitpython-motorkit`
  - `adafruit-circuitpython-motor`

### Remote Server

- **Hardware:**
  - A server machine with sufficient resources to handle 3D reconstruction
- **Software:**
  - Python 3.x
  - [COLMAP](https://colmap.github.io/) installed and accessible via PATH

- **Python Packages:**
  - `opencv-python`
  - `pyyaml`
  - `tqdm`

### Point Cloud Analysis

- **Python Packages:**
  - `numpy`
  - `open3d`
  - `matplotlib`
  - `scipy`
  - `scikit-learn`
  - `networkx`
  - `alphashape`
  - `shapely`


## Installation

### Main System Installation

1. **Clone the Repository:**

    ```bash
    git clone https://github.com/joehrz/PhotoPi.git
    cd PhotoPi
    ```

2. **Create a Virtual Environment (Optional but Recommended):**

    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows use `venv\Scripts\activate`
    ```


3. **Install Additional Components:**

    To install all components (main system, remote server, Raspberry Pi, and point cloud analysis), use:

    ```bash
    pip install -e .[all]
    ```

    For example, installing specific components:

    - **Remote Server:**

        ```bash
        pip install -e .[remote_server]
        ```
### Raspberry Pi Installation

1. **Ensure your Raspberry Pi is up to date:**

    ```bash
    sudo apt-get update
    sudo apt-get upgrade
    ```

2. **Set up the 64MP Autofocus Synchronized Quad-Camera Kit:**

    You can set up the camera kit manually or use the `deploy_to_pi.sh` script to automate the process.

    #### Manual Setup:

    Run the following commands on your Raspberry Pi:

    ```bash
    wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
    chmod +x install_pivariety_pkgs.sh

    # Install libcamera dev and apps
    ./install_pivariety_pkgs.sh -p libcamera_dev
    ./install_pivariety_pkgs.sh -p libcamera_apps

    # Install the Hawk-Eye kernel driver
    ./install_pivariety_pkgs.sh -p 64mp_pi_hawk_eye_kernel_driver
    ```

    Then, edit the `/boot/config.txt` file and add the following line under the `[pi4]` section:

    ```bash
    dtoverlay=vc4-kms-v3d,cma-512
    ```

    #### Automated Setup:

    Run the `deploy_to_pi.sh` script, which will handle the setup and deploy the code to the Raspberry Pi:

    ```bash
    ./deploy_to_pi.sh
    ```

    This script will automatically set up the necessary drivers, configure the system, and deploy the project files.

## Usage

### Launching the Application

1. **Ensure Your Raspberry Pi is Connected and Set Up:**
   - Power on the Raspberry Pi.
   - Verify that all cameras and the turntable are properly connected.

2. **Run the GUI Application on the Main System:**

    ```bash
    python photopack/main_system/gui.py
    ```

    Alternatively, if you installed the package with entry points:

    ```bash
    photopack-main
    ```

### Main System Operations

Using the GUI, you can:

- **Configure Camera Settings:** Adjust angles and image capture parameters.
- **Capture Images:** Initiate image capture sequences.
- **Inspect and Manage Images:** View captured images and organize them as needed.
- **Transfer Images:** Download images from the Raspberry Pi to your local system or remote server for further processing.


### Remote Server Operations

The Remote Server component is responsible for running Structure from Motion (SfM) photogrammetry using COLMAP to reconstruct 3D models from images captured by the Raspberry Pi.

1. **Deploy to Remote Server:**

    ```bash
    ./deploy_to_remote_server.sh
    ```

    Alternatively, if installed via entry points:

    ```bash
    photopack-remote-server
    ```

2. **Features:**
   - **Photogrammetry Processing:** Images taken from the Raspberry Pi system is process using COLMAP to generate 3D models (point clouds).
   - **Automated Workflow:** Streamlines the process of image, processing, and 3D model generation.
   - **Integration:** Integrates the photogrammetry results into existing workflows, enabling further analysis and visualization.

3. **Usage:**
   - Ensure that the remote server has COLMAP installed and accessible via the system PATH.
   - Configure the remote server settings in the `params.json` file.
   - Run the remote server script to start processing images and 3D reconstruct point clouds with COLMAP.


### Point Cloud Analysis

PhotoPi includes tools for analyzing 3D models generated from captured images.

1. **Run Point Cloud Analysis:**

    ```bash
    photopack-analyze <path_to_point_cloud_file> [options]
    ```

    For example:

    ```bash
    photopack-analyze models/plant_model.ply --diameter 1.5 --module convex_hull leaf_angles
    ```

2. **Available Modules:**

    - **processing:** General point cloud processing.
    - **convex_hull:** Computes the convex hull volume of the plant model.
    - **hr_analysis:** Performs height/radius analysis.
    - **leaf_angles:** Measures leaf angles.
    - **projection:** Analyzes projections and areas.
    - **all:** Runs all available modules.

3. **Example Usage:**

    ```bash
    python -m photopack.point_cloud_analysis.main models/plant_model.ply --module all
    ```

    This command will run all analysis modules on the specified point cloud file.


### Raspberry Pi

To control and capture images directly from the Raspberry Pi (optional, depending on your setup), use the provided scripts in the `raspberry_pi/` directory.

## Configuration

Configuration settings are managed through the params.json file located in the photopack/main_system/ directory. You can modify this file directly or use the GUI to adjust settings.






