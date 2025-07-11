# PhotoPi/photopack/main_system/constants.py
"""
Constants and configuration values for the PhotoPi system.
This file centralizes all magic numbers and configuration constants.
"""

# Camera Configuration
CAMERA_I2C_CODES = {
    'A': '0x32',
    'B': '0x22', 
    'C': '0x12',
    'D': '0x02'
}

# Camera Capture Settings
CAMERA_CAPTURE_TIMEOUT_MS = 5000  # milliseconds
CAMERA_SHARPNESS = 2.0
CAMERA_VIEWFINDER_WIDTH = 2312
CAMERA_VIEWFINDER_HEIGHT = 1736
CAMERA_OUTPUT_WIDTH = 4056
CAMERA_OUTPUT_HEIGHT = 3040
CAMERA_ROI = "0.25,0.25,0.45,0.45"  # Region of Interest
CAMERA_FOCAL_LENGTH = "51/10"  # EXIF focal length
CAMERA_F_NUMBER = "9/5"  # EXIF F-number

# Turntable Configuration
TURNTABLE_STEPS_PER_DEGREE = 1 / 0.18  # Steps required for 1 degree rotation
TURNTABLE_STEP_DELAY = 0.01  # seconds between steps

# Network Configuration
SSH_RECONNECT_DELAY = 60  # seconds to wait before reconnecting after reboot
SSH_RECONNECT_RETRY_INTERVAL = 5  # seconds between reconnection attempts
DEFAULT_PI_PROJECT_DIR = '/home/pi/PhotoPi'

# GUI Configuration
GUI_WINDOW_WIDTH = 500
GUI_WINDOW_HEIGHT = 500
GUI_UPDATE_DELAY_MS = 100  # milliseconds for thread-safe GUI updates

# Image Processing
IMAGE_THUMBNAIL_MAX_SIZE = (800, 600)
IMAGE_FILE_EXTENSIONS = ('.jpg', '.jpeg', '.png')

# Validation Limits
ANGLE_MIN = 0
ANGLE_MAX = 360
PLANT_NAME_MAX_LENGTH = 100
FOLDER_PATH_MAX_LENGTH = 500

# I2C Configuration
I2C_BUS_NUMBER = 10
I2C_DEVICE_ADDRESS = 0x24
I2C_REGISTER_ADDRESS = 0x24