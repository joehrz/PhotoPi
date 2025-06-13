# PhotoPi/photopack/main_system/imagecapture.py
"""
This module handles the core logic for capturing images using the PhotoPi system.
It defines the CameraSystem class, which communicates with the Raspberry Pi via SSH
to control the cameras and turntable, and manages the transfer of captured images
back to the main computer.
"""

import paramiko
import json
import os
import io
import logging
from PIL import Image, ImageTk
import tkinter as tk
from dotenv import load_dotenv
import time

# Load environment variables from a .env file for easy configuration.
load_dotenv()

# Configure logging to provide status updates and error information.
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class ConfigLoader:
    """A simple helper class to load configuration from a JSON file."""
    def __init__(self, config_path='params.json'):
        """
        Initializes the ConfigLoader.

        Args:
            config_path (str): The path to the JSON configuration file.
        """
        self.config_path = config_path

    def load(self):
        """
        Loads and returns the configuration data from the JSON file.

        Returns:
            dict: A dictionary containing the configuration data, or an empty dict on error.
        """
        try:
            with open(self.config_path, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            logging.error(f"Error loading configuration from {self.config_path}: {e}")
            return {}

class SSHClientWrapper:
    """A wrapper for the paramiko SSHClient to simplify command execution and file transfers."""
    def __init__(self, ssh_client):
        """
        Initializes the wrapper with an active paramiko SSHClient instance.

        Args:
            ssh_client: An authenticated paramiko.SSHClient object.
        """
        self.ssh_client = ssh_client

    def execute_command(self, command):
        """
        Executes a shell command on the remote machine.

        Args:
            command (str): The command to execute.

        Returns:
            str: The standard output from the command, or an empty string on error.
        """
        try:
            stdin, stdout, stderr = self.ssh_client.exec_command(command)
            output = stdout.read().decode()
            error = stderr.read().decode()
            if error:
                logging.warning(f"Remote command stderr: {error.strip()}")
            return output
        except Exception as e:
            logging.error(f"SSH command execution failed: {e}")
            return ""

    def transfer_files(self, remote_path, local_path):
        """
        Transfers all files from a remote directory to a local directory using SFTP.

        Args:
            remote_path (str): The source directory on the remote machine.
            local_path (str): The destination directory on the local machine.
        """
        try:
            sftp_client = self.ssh_client.open_sftp()
            os.makedirs(local_path, exist_ok=True)
            file_list = sftp_client.listdir(remote_path)
            for file_name in file_list:
                remote_file = os.path.join(remote_path, file_name)
                local_file = os.path.join(local_path, file_name)
                sftp_client.get(remote_file, local_file)
            sftp_client.close()
            logging.info(f"Files transferred from {remote_path} to {local_path}")
        except Exception as e:
            logging.error(f"File transfer from {remote_path} failed: {e}")

class CameraSystem:
    """Manages all camera and turntable operations on the Raspberry Pi."""
    def __init__(self, ssh_client, gui_root, config_loader):
        """
        Initializes the CameraSystem.

        Args:
            ssh_client: An authenticated paramiko.SSHClient object.
            gui_root (tk.Tk): The root Tkinter window for displaying image pop-ups.
            config_loader (ConfigLoader): An instance of ConfigLoader to get parameters.
        """
        self.gui_root = gui_root
        self.ssh_client = SSHClientWrapper(ssh_client)
        self.config_loader = config_loader
        self.config = self.config_loader.load()
        
        self.pi_project_dir = os.getenv('PI_PROJECT_DIR', '/home/pi/PhotoPi')

        self.image_frame = tk.Frame(self.gui_root)
        self.image_frame.grid(row=0, column=0)
        self.image_index = 0
        self.images = []
        self.image_label = None
        self.camera_labels = {}
    
    def angle_to_steps(self, angle):
        return int(angle * (1 / 0.18))

    def full_rev_count(self, angle):
        if angle == 0:
            return 0
        return 360 // angle

    def inspect(self):
        """Captures and displays a test image from each selected camera."""
        folder_with_date = self.config.get("folder_with_date")
        plant_folder = os.path.basename(folder_with_date) if folder_with_date else 'default_folder'
        
        remote_inspect_path = os.path.join(self.pi_project_dir, 'Images', plant_folder, 'inspect')
        self.ssh_client.execute_command(f'sudo mkdir -p {remote_inspect_path}')

        for camera_id in ['A', 'B', 'C', 'D']:
            if self.config.get(f"camera_{camera_id.lower()}", 0) == 1:
                self.capture_image(plant_folder, self.config.get("plant_name", "Unknown"), "inspect", camera_id, 'inspect')
        
        self.fetch_and_display_images(remote_inspect_path)

    def imaging(self):
        """Runs the full, automated imaging sequence: rotate, capture, and finally transfer."""
        folder_with_date = self.config.get("folder_with_date")
        plant_folder = os.path.basename(folder_with_date) if folder_with_date else 'default_folder'
        
        remote_images_path = os.path.join(self.pi_project_dir, 'Images', plant_folder, 'images')
        self.ssh_client.execute_command(f'sudo mkdir -p {remote_images_path}')

        ANGLE = int(self.config.get("angle"))
        SECONDS = int(self.config.get("seconds"))

        steps = self.angle_to_steps(ANGLE)
        count = self.full_rev_count(ANGLE)

        for j in range(count):
            time.sleep(SECONDS)

            for camera_id in ['A', 'B', 'C', 'D']:
                if self.config.get(f"camera_{camera_id.lower()}", 0) == 1:
                    timestamp = f"{self.config.get('Dates', '20240101')}_{j:03d}"
                    self.capture_image(plant_folder, self.config.get("plant_name", "Unknown"), timestamp, camera_id, 'images')

            turntable_script_path = os.path.join(self.pi_project_dir, 'turntable.py')
            cmd = f'python {turntable_script_path} {steps}'
            self.ssh_client.execute_command(cmd)

        # After the loop, transfer the 'images' folder
        self.transfer_images(plant_folder, 'images')

    def capture_image(self, plant_folder, plant_name, the_time, camera_id, image_folder):
        """Executes the command on the Pi to capture a single image and move it to the correct folder."""
        file_name = f"{plant_name}_Camera_{camera_id}_{the_time}.jpg"
        remote_image_destination = os.path.join(self.pi_project_dir, 'Images', plant_folder, image_folder)

        command = (
            f'i2cset -y 10 0x24 0x24 {self.get_camera_code(camera_id)}; '
            f'libcamera-jpeg --sharpness 2.0 -t 5000 --viewfinder-width 2312 '
            f'--viewfinder-height 1736 --width 4056 --height 3040 --roi 0.25,0.25,0.45,0.45 '
            f'--vflip '
            f'-o {file_name} --exif EXIF.FocalLength=51/10 '
            f'--autofocus-on-capture --autofocus-mode auto '
            f'--exif EXIF.FNumber=9/5; '
            f'sudo mv {file_name} {remote_image_destination}'
        )
        self.ssh_client.execute_command(command)
        logging.info(f"Camera {camera_id} imaging done. Image saved to {remote_image_destination}")
        self.camera_labels[file_name] = camera_id

    def get_camera_code(self, camera_id):
        """Returns the I2C code required to select a specific camera on the multiplexer."""
        camera_codes = {'A': '0x32', 'B': '0x22', 'C': '0x12', 'D': '0x02'}
        return camera_codes.get(camera_id, '0x32')
    
    def fetch_and_display_images(self, image_directory):
        """Fetches all images from a remote directory and displays them in a new GUI window."""
        raw_images = self.fetch_images(image_directory)
        self.images = [(self.resize_image(img_data), file_name) for img_data, file_name in raw_images]
        self.images.sort(key=lambda x: self.camera_labels.get(x[1], "Unknown"))
        self.create_image_window()

    def fetch_images(self, image_directory):
        """Fetches raw image data from a specified directory on the Pi using SFTP."""
        raw_images = []
        sftp_client = None
        try:
            sftp_client = self.ssh_client.ssh_client.open_sftp()
            file_list = sftp_client.listdir(image_directory)
            for file_name in file_list:
                if file_name.lower().endswith(('.png', '.jpg', '.jpeg')):
                    file_path = os.path.join(image_directory, file_name)
                    with sftp_client.open(file_path, 'rb') as file_handle:
                        raw_images.append((file_handle.read(), file_name))
        except Exception as e:
            logging.error(f"An error occurred while fetching images from {image_directory}: {e}")
        finally:
            if sftp_client and not sftp_client.sock.closed:
                sftp_client.close()
        return raw_images

    def resize_image(self, image_data):
        """Resizes an image to a thumbnail for display in the GUI."""
        max_size = (800, 600)
        image = Image.open(io.BytesIO(image_data))
        image.thumbnail(max_size, Image.Resampling.LANCZOS)
        return ImageTk.PhotoImage(image)

    def create_image_window(self):
        """Creates the pop-up window for navigating through inspection images."""
        if not self.images:
            logging.info("No images to display.")
            return

        window = tk.Toplevel(self.gui_root)
        window.title("Image Inspection")
        self.image_label = tk.Label(window)
        self.image_label.pack()

        self.camera_info_label = tk.Label(window, text="")
        self.camera_info_label.pack()

        tk.Button(window, text="<< Previous", command=self.show_previous_image).pack(side="left")
        tk.Button(window, text="Next >>", command=self.show_next_image).pack(side="right")

        self.show_image(0)

    def show_image(self, index):
        """Displays a specific image in the inspection window by its index."""
        if 0 <= index < len(self.images):
            self.image_index = index
            image, file_name = self.images[index]
            self.image_label.config(image=image)
            camera_id = self.camera_labels.get(file_name, "Unknown")
            self.camera_info_label.config(text=f"Image: {file_name} (Camera {camera_id})")

    def show_next_image(self):
        """Shows the next image in the sequence."""
        if self.image_index < len(self.images) - 1:
            self.show_image(self.image_index + 1)

    def show_previous_image(self):
        """Shows the previous image in the sequence."""
        if self.image_index > 0:
            self.show_image(self.image_index - 1)

    def transfer_images(self, plant_folder, image_folder):
        """
        Transfers the completed set of images from the Pi back to the local machine,
        preserving the final directory structure.
        """
        # Get the full local path for the run, e.g., '.../images/test5_2025-06-13-1331'
        base_local_run_path = self.config.get("folder_with_date")
        if not base_local_run_path:
            logging.error("Could not determine local folder path from config.")
            return
            
        # Create the final local directory, mirroring the remote 'images' or 'inspect' subfolder.
        # e.g., '.../test5_2025-06-13-1331/images'
        final_local_dir = os.path.join(base_local_run_path, image_folder)

        # Construct the full remote directory path to transfer from.
        remote_dir = os.path.join(self.pi_project_dir, 'Images', plant_folder, image_folder)
        
        # The transfer_files function will create the final_local_dir if it doesn't exist.
        self.ssh_client.transfer_files(remote_dir, final_local_dir)


if __name__ == "__main__":
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        pi_hostname = os.getenv('PI_HOST', 'raspberrypi.local')
        pi_username = os.getenv('PI_USERNAME')
        pi_password = os.getenv('PI_PASSWORD')
        
        if not all([pi_hostname, pi_username, pi_password]):
            raise ValueError("PI_HOST, PI_USERNAME, and PI_PASSWORD must be set in your .env file")

        client.connect(pi_hostname, username=pi_username, password=pi_password)
        
        root = tk.Tk()
        root.withdraw()
        
        config_loader = ConfigLoader()
        camera_system = CameraSystem(client, root, config_loader)
        
        logging.info("CameraSystem initialized successfully for testing.")

    except Exception as e:
        logging.error(f"Failed to initialize for testing: {e}")
    finally:
        client.close()