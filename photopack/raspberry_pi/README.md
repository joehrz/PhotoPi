# PhotoPi: Raspberry Pi System

This directory contains Python scripts and information specific to the Raspberry Pi component of the PhotoPi system. The Raspberry Pi is responsible for direct hardware control, including camera operation and turntable movement.

For a general overview of the entire PhotoPi project, please see the [main README.md](../../README.md).

## Table of Contents

* [Overview](#overview)
* [Hardware Requirements & Setup](#hardware-requirements--setup)
    * [Camera Hardware Setup](#camera-hardware-setup)
* [Software and Dependencies](#software-and-dependencies)
    * [Camera Driver Installation](#camera-driver-installation)
* [Deployment](#deployment)
* [Operation](#operation)
    * [Camera Operation](#camera-operation)
* [Key Scripts](#key-scripts)
    * [`turntable.py`](#turntablepy)
* [Troubleshooting](#troubleshooting)
    * [Camera Safety Precautions](#camera-safety-precautions)

## Overview

The Raspberry Pi acts as the embedded control unit for the PhotoPi image acquisition hardware. It receives commands and configuration (via a `params.json` file) from the Main System (Control PC) over SSH. Based on these, it triggers the attached cameras to capture images and controls a stepper motor to rotate the turntable.

## Hardware Requirements & Setup

* **Raspberry Pi:** Raspberry Pi 4 Model B is recommended.
* **Cameras:** Arducam 64MP Autofocus Quad-Camera Kit. The kit includes:
    * 4 x 64MP Autofocus Camera Modules
    * 1 x Quad-Camera HAT
    * Flex cables, screws, spacers, and nuts
* **Motor Controller:** Adafruit Stepper Motor HAT (or a similar board compatible with `adafruit_motorkit`).
* **Stepper Motor & Turntable:** A stepper motor connected to the Motor HAT, driving a physical turntable.
* **Power:** Adequate power supply for the Raspberry Pi and the Motor HAT (stepper motors can draw significant current).
* **Network:** Ethernet or WiFi connection.

**Wiring:** Refer to the main project [Wiring Diagram](../../README.md#wiring-diagram) for crucial connections between the Raspberry Pi, Motor HAT, and stepper motor. Camera connections are detailed below.

### Camera Hardware Setup

**Important:** The 64MP camera modules in the kit must be used with the included Quad-Camera HAT and cannot be connected directly to the Raspberry Pi.

1.  **Power Down:** Always turn off your Raspberry Pi and disconnect the power supply before connecting any components.
2.  **Attach HAT to Pi:** Connect the Quad-Camera HAT's `MIPI_TXO` port to the Raspberry Pi's camera port (MIPI CSI-2) using the provided flex cable. Secure the HAT to the Raspberry Pi board using the included screws and spacers.
3.  **Connect Cameras to HAT:** Connect the four camera modules to the `Rx` ports on the Quad-Camera HAT using the smaller flex cables. Ensure the cables are securely locked in place.
4.  **Power Up:** Once all hardware is securely connected, you can power your Raspberry Pi on.

## Software and Dependencies

* **OS:** A recent version of Raspberry Pi OS is required (releases from 01/28/22 or later). A fresh installation is highly recommended.
* **Python:** Python 3.x.
* **Camera Stack:** `libcamera-apps` and Arducam Pivariety drivers are required. See the installation steps below.
* **Python Packages:**
    * `RPi.GPIO` (for general GPIO interaction, though `adafruit_motorkit` handles motor pins).
    * `adafruit-circuitpython-motorkit`
    * `adafruit-circuitpython-motor`
    These should be installed on the Raspberry Pi.

### Camera Driver Installation

After setting up the hardware and installing a fresh Raspberry Pi OS, you must install the necessary drivers and configure the system.

1.  **Download Installation Script**:
    ```bash
    wget -O install_pivariety_pkgs.sh [https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh](https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh)
    chmod +x install_pivariety_pkgs.sh
    ```
2.  **Install `libcamera`**:
    ```bash
    ./install_pivariety_pkgs.sh -p libcamera_dev
    ./install_pivariety_pkgs.sh -p libcamera_apps
    ```
3.  **Install Kernel Driver**:
    ```bash
    ./install_pivariety_pkgs.sh -p 64mp_pi_hawk_eye_kernel_driver
    ```
4.  **Update Boot Configuration**:
    Open the `/boot/config.txt` file and add the following line under the `[all]` section to increase the memory available to the camera system:
    ```
    dtoverlay=vc4-kms-v3d,cma-512
    ```
5.  **Reboot** your Raspberry Pi to apply the changes.

## Deployment

The recommended method for setting up the Raspberry Pi and deploying the necessary PhotoPi scripts is by using the `deploy_to_pi.sh` script located in the root of the PhotoPi repository. This script automates the transfer of your project files.

```bash
# Execute this from your Control PC, in the PhotoPi project root directory:
./deploy_to_pi.sh
```

## Operation

By default, the quad-camera kit works in a synchronized 4-channel mode, and the Raspberry Pi recognizes the whole kit as one camera. Any manual focus or camera control adjustments (exposure/gain/white balance/etc.) will also be applied to all 4 cameras at the same time.

### Camera Operation

* **List Available Cameras**:
    ```bash
    libcamera-still --list-cameras
    ```

* **Preview Camera Feed**:
    For a balance of quality and performance, a preview resolution of 2312x1736 is recommended.
    ```bash
    libcamera-still -t 0 --viewfinder-width 2312 --viewfinder-height 1736
    ```

* **Capture an Image with Autofocus**:
    ```bash
    # For Raspberry Pi 4
    libcamera-still -t 5000 --viewfinder-width 2312 --viewfinder-height 1736 -o pi_hawk_eye.jpg --autofocus
    ```

* **Manual Focus Control**:
    * Clone the driver repository and navigate to the focus directory:
        ```bash
        git clone [https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver.git](https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver.git)
        cd Arducam-Pivariety-V4L2-Driver/focus
        ```
    * Run the focuser script:
        ```bash
        python3 FocuserExample.py -d /dev/v4l-subdev1
        ```
    * Press the Up/Down Arrow for focus adjustment, press "ctrl + c" to save, or "r" to reset. You can change the step size with the `--focus-step [number]` argument. The default step is 50, and the range is 1-1023.

* **Switching Camera Channels**:
    You can switch between single, dual, or the default four-channel modes using `i2cset` commands.
    ```bash
    # Revert to default four-in-one mode
    i2cset -y 10 0x24 0x24 0x00
    ```
## Key Scripts

#### `turntable.py`

This script controls the stepper motor via the Adafruit Motor HAT to rotate the turntable.