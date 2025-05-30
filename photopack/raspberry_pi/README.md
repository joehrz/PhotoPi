# PhotoPi: Raspberry Pi System

This directory contains Python scripts and information specific to the Raspberry Pi component of the PhotoPi system. The Raspberry Pi is responsible for direct hardware control, including camera operation and turntable movement.

For a general overview of the entire PhotoPi project, please see the [main README.md](../../README.md).

## Table of Contents

* [Overview](#overview)
* [Hardware Requirements & Setup](#hardware-requirements--setup)
* [Software and Dependencies](#software-and-dependencies)
* [Deployment](#deployment)
* [Operation](#operation)
* [Key Scripts](#key-scripts)
    * [`turntable.py`](#turntablepy)
* [Troubleshooting](#troubleshooting)

## Overview

The Raspberry Pi acts as the embedded control unit for the PhotoPi image acquisition hardware. It receives commands and configuration (via a `params.json` file) from the Main System (Control PC) over SSH. Based on these, it triggers the attached cameras to capture images and controls a stepper motor to rotate the turntable.

## Hardware Requirements & Setup

* **Raspberry Pi:** Raspberry Pi 4 Model B is recommended.
* **Cameras:** Arducam 64MP Autofocus Quad-Camera Kit (or a compatible multi-camera setup).
* **Motor Controller:** Adafruit Stepper Motor HAT (or a similar board compatible with `adafruit_motorkit`).
* **Stepper Motor & Turntable:** A stepper motor connected to the Motor HAT, driving a physical turntable.
* **Power:** Adequate power supply for the Raspberry Pi and the Motor HAT (stepper motors can draw significant current).
* **Network:** Ethernet or WiFi connection.

**Wiring:** Refer to the main project [Wiring Diagram](../../README.md#wiring-diagram) for crucial connections between the Raspberry Pi, Motor HAT, and stepper motor. Camera connections are typically made to the MIPI CSI ports.

## Software and Dependencies

* **OS:** Raspberry Pi OS (latest version recommended, based on Debian Bullseye or newer for `libcamera`).
* **Python:** Python 3.x.
* **Camera Stack:** `libcamera-apps` and Arducam Pivariety drivers. These are typically installed using the `deploy_to_pi.sh` script or Arducam's official installation scripts.
* **Python Packages:**
    * `RPi.GPIO` (for general GPIO interaction, though `adafruit_motorkit` handles motor pins).
    * `adafruit-circuitpython-motorkit`
    * `adafruit-circuitpython-motor`
    These should be installed on the Raspberry Pi.

## Deployment

The recommended method for setting up the Raspberry Pi and deploying the necessary PhotoPi scripts is by using the `deploy_to_pi.sh` script located in the root of the PhotoPi repository.

```bash
# Execute this from your Control PC, in the PhotoPi project root directory:
./deploy_to_pi.sh