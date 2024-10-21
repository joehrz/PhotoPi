# PhotoPi

**PhotoPi** is designed for plant photogrammetry, providing a GUI to automate the capture of 2D images used in constructing 3D models of plants. The system controls four cameras and a turntable via a Raspberry Pi, ensuring precise control over the imaging process. The application is operated remotely from a main computer, integrating into photogrammetry workflows. The imaging system is a Python-based GUI application designed to manage camera systems via SSH, specifically for Raspberry Pi setups. It allows users to control camera settings, capture images, and manage these images remotely. The project utilizes Tkinter for the GUI and Paramiko for SSH communication, making it a robust tool for remote camera management.





## Features

- **Dynamic SSH Connection Management:** Connect and disconnect from Raspberry Pi systems over SSH.
- **User Authentication and Network Configuration:** Simple GUI-based authentication for connecting to Raspberry Pi, with support for configuring network settings.
- **Live Camera Control and Image Capture:** Direct control of multiple cameras, with options to capture images and save them to the Raspberry Pi or download them locally.
- **Configurable Settings:** Configuration for camera angles, image capture timings, and more, to suit various use cases.
- **Secure Credential Handling:** Safe and secure management of user credentials using environment variables.
- **Inspection and Management:** Inspect captured images through the GUI and manage them efficiently.
- **Reset Function:** Reset the system in case of failures.

## Requirements

### Main System

- Python 3.x
- Required Python packages (listed in `main_system/requirements.txt`)

### Raspberry Pi

- Raspberry Pi with camera(s) attached
- 64MP Autofocus Synchronized Quad-Camera Kit for Raspberry Pi (if using multiple cameras)
- Python 3.x

## Installation

### Main System

1. **Clone the repository:**

    ```bash
    git clone https://github.com/Dxxc/PhotoPi.git
    cd PhotoPi
    ```

2. **Create a virtual environment (optional but recommended):**

    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows use `venv\Scripts\activate`
    ```

3. **Install the required packages:**

    ```bash
    pip install -r main_system/requirements.txt
    ```

### Raspberry Pi

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

### Main System

1. **Ensure your Raspberry Pi is connected to the network and the cameras are properly set up.**
2. **Run the `gui.py` script to launch the application:**

    ```bash
    python main_system/gui.py
    ```

3. **Use the GUI to:**

    - Configure camera settings.
    - Capture images.
    - Inspect and manage images.
    - Transfer images from the Raspberry Pi to the local system.

### Raspberry Pi

To control and capture images directly from the Raspberry Pi (optional, depending on your setup), use the provided scripts in the `raspberry_pi/` directory.

## Configuration

Configuration settings are managed through a JSON file (`params.json`). You can update this file directly or use the GUI to modify settings.

## File Structure

```plaintext
PhotoPi/
├── README.md
├── setup.py
├── requirements.txt
├── deploy_to_pi.sh
├── deploy_to_remote_server.sh
<<<<<<< HEAD
└── photopack/                      # Top-level package
=======
>>>>>>> 2fab5a5c6d700ddba425a173138e61c6a82421f8
    ├── __init__.py
    ├── main_system/
    │   ├── __init__.py
    │   ├── gui.py
    │   ├── network.py
    │   ├── credentials.py
    │   ├── config.py
    │   ├── imagecapture.py
    │   ├── params.json
    │
    ├── remote_server/
    │   ├── __init__.py
    │   ├── build.py
    │   ├── config.yaml
    │
    ├── raspberry_pi/
    │   ├── __init__.py
    │   ├── turntable.py
    │
    └── point_cloud_analysis/
        ├── __init__.py
        ├── main.py
        ├── point_cloud/
        │   ├── __init__.py
        │   ├── convex_hull.py
        │   ├── processing.py
        │   ├── hr_analysis.py
        │   ├── projection.py
        │   ├── leaf_angles.py
        │
        └── utils/
            ├── __init__.py
            └── helpers.py




