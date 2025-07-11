# PhotoPi/photopack/main_system/gui.py

import tkinter as tk
from tkinter import filedialog
import time
import json
from datetime import datetime, timedelta
import paramiko
import logging
import threading
from tkinter import messagebox
import os
from dotenv import load_dotenv
from pathlib import Path
from typing import Dict, Any, Optional

from .imagecapture import CameraSystem
from .credentials import NetworkConfig
from .config import Config
from .unified_network import UnifiedNetworkManager
from .security import (
    SecureHostKeyPolicy, sanitize_filename, sanitize_path,
    validate_plant_name, validate_angle, validate_positive_integer
)
from .constants import (
    GUI_WINDOW_WIDTH, GUI_WINDOW_HEIGHT, GUI_UPDATE_DELAY_MS,
    SSH_RECONNECT_DELAY, SSH_RECONNECT_RETRY_INTERVAL,
    ANGLE_MIN, ANGLE_MAX, PLANT_NAME_MAX_LENGTH, FOLDER_PATH_MAX_LENGTH
)


# Load environment variables from a .env file
load_dotenv()

# Configure the root logger in the main module
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class InputGUI:

    def __init__(self, master):
        """
        Initializes the InputGUI class, sets up the main window, and initializes network and SSH connections.

        Parameters:
            master (tk.Tk): The main window of the Tkinter application.
        """
        self.master = master
        self.master.title("PhotoPI")
        self.master.geometry(f"{GUI_WINDOW_WIDTH}x{GUI_WINDOW_HEIGHT}")
        self.config = Config()
        self.network_manager = None
        self.ssh_client = paramiko.SSHClient()
        # Use secure host key policy instead of AutoAddPolicy
        self.ssh_client.set_missing_host_key_policy(SecureHostKeyPolicy())

        # Load credentials from environment variables
        self.pi_username = os.getenv('PI_USERNAME')
        self.pi_password = os.getenv('PI_PASSWORD')


        if self.initialize_network_and_ssh():
            self.setup_gui_components()
        else:
            self.master.quit()

    def initialize_network_and_ssh(self):
        """
        Initializes network settings and SSH client using dynamic credentials.

        Returns:
            bool: True if the network and SSH connection are successfully established, False otherwise.
        """
        net_config = NetworkConfig(self.master)
        try:
            net_config.discover_pi_hostname()

            if net_config.hostname and self.pi_username and self.pi_password:
                self.network_manager = UnifiedNetworkManager(net_config.hostname, self.pi_username, self.pi_password)
                self.config.set_value('pi_hostname', net_config.hostname)
                self.config.save_config()

                self.ssh_client.connect(net_config.hostname, username=self.pi_username, password=self.pi_password)
                logging.info("Network and SSH connection established successfully.")
                return True
            else:
                raise ValueError("Hostname, username, or password not provided or incorrect.")
        except (paramiko.ssh_exception.NoValidConnectionsError, ValueError) as e:
            messagebox.showerror("Connection Error", f"Failed to establish network or SSH connection: {str(e)}")
            return False

    def update_gui_from_thread(self, message):
        """
        Safely updates the GUI from a background thread.

        Parameters:
            message (str): The message to display in the GUI.
        """
        def update_status():
            if "successfully" in message:
                messagebox.showinfo("Success", message)
            else:
                messagebox.showerror("Error", message)
        self.master.after(GUI_UPDATE_DELAY_MS, update_status)
        


    def validate_angle_entry(self, text: str) -> bool:
        """Validate angle entry allowing empty string or valid angle."""
        if text == "":
            return True
        try:
            value = int(text)
            return ANGLE_MIN <= value <= ANGLE_MAX
        except ValueError:
            return False

    def validate_text_entry(self, text: str) -> bool:
        """
        Validates text entry for plant names.

        Parameters:
            text (str): The text to validate.

        Returns:
            bool: True if valid, False otherwise.
        """
        if len(text) > PLANT_NAME_MAX_LENGTH:
            return False
        # Allow alphanumeric, spaces, hyphens, and underscores
        if text and not all(c.isalnum() or c in ' -_' for c in text):
            return False
        return True
        
    def setup_gui_components(self):
        """
        Sets up the GUI components including camera checkbuttons, plant name entry, folder path entry, and action buttons.
        """
        self.setup_camera_checkbuttons()
        self.setup_plant_name_entry()
        self.setup_folder_path_entry()
        self.setup_turntable_angle_entry()
        self.setup_image_delay_entry()
        self.setup_action_buttons()

    def setup_camera_checkbuttons(self):
        """
        Sets up the camera checkbuttons in the GUI.
        """
        self.camera_label = tk.Label(self.master, text="Cameras:")
        self.camera_label.grid(row=0, column=1)

        self.camera_a_var = tk.IntVar(value=0)
        self.camera_a_checkbutton = tk.Checkbutton(self.master, text="Camera A", variable=self.camera_a_var)
        self.camera_a_checkbutton.grid(row=1, column=1)

        self.camera_b_var = tk.IntVar(value=0)
        self.camera_b_checkbutton = tk.Checkbutton(self.master, text="Camera B", variable=self.camera_b_var)
        self.camera_b_checkbutton.grid(row=2, column=1)

        self.camera_c_var = tk.IntVar(value=0)
        self.camera_c_checkbutton = tk.Checkbutton(self.master, text="Camera C", variable=self.camera_c_var)
        self.camera_c_checkbutton.grid(row=3, column=1)

        self.camera_d_var = tk.IntVar(value=0)
        self.camera_d_checkbutton = tk.Checkbutton(self.master, text="Camera D", variable=self.camera_d_var)
        self.camera_d_checkbutton.grid(row=4, column=1)
    
    def setup_plant_name_entry(self):
        """
        Sets up the plant name entry field in the GUI.
        """
        tcmd = (self.master.register(self.validate_text_entry), "%P")

        self.plant_name_label = tk.Label(self.master, text="Plant Name:")
        self.plant_name_label.grid(row=5)
        self.plant_name_entry = tk.Entry(self.master, validate="key", validatecommand=tcmd)
        self.plant_name_entry.grid(row=5, column=1)

    def setup_turntable_angle_entry(self):
        """
        Sets up the plant name entry field in the GUI.
        """
        acmd = (self.master.register(self.validate_angle_entry), "%P")
        
        self.angle_label = tk.Label(self.master, text="Angle:")
        self.angle_label.grid(row=6)
        self.angle_entry = tk.Entry(self.master, validate="key", validatecommand=acmd)
        self.angle_entry.grid(row=6, column=1)

    def setup_image_delay_entry(self):
        """
        Sets up the plant name entry field in the GUI.
        """
        acmd = (self.master.register(self.validate_angle_entry), "%P")
        
        self.seconds_label = tk.Label(self.master, text="Image Capture Delay:")
        self.seconds_label.grid(row=7)
        self.seconds_entry = tk.Entry(self.master, validate="key", validatecommand=acmd)
        self.seconds_entry.grid(row=7, column=1)

    def setup_folder_path_entry(self):
        """
        Sets up the folder path entry field and browse button in the GUI.
        """
        tk.Label(self.master, text="Folder Path:").grid(row=8)
        self.folder_path_entry = tk.Entry(self.master)
        self.folder_path_entry.grid(row=8, column=1)
        tk.Button(self.master, text="Browse", command=self.browse_folder).grid(row=8, column=2)

    def setup_action_buttons(self):
        """
        Sets up action buttons (Submit, Inspect Images, Start Imaging, Reset, Quit) in the GUI.
        """
        self.submit_button = tk.Button(self.master, text="Submit", command=self.submit)
        self.submit_button.grid(row=9, column=0)

        self.inspect_button = tk.Button(self.master, text="Inspect Images", command=self.perform_inspection)
        self.inspect_button.grid(row=10, column=0)

        self.start_imaging_button = tk.Button(self.master, text="Start Imaging", command=self.start_imaging)
        self.start_imaging_button.grid(row=10, column=1)

        self.reset_button = tk.Button(self.master, text="Reset", command=self.reset)
        self.reset_button.grid(row=10, column=2)

        self.quit_button = tk.Button(self.master, text="Quit", command=self.master.quit)
        self.quit_button.grid(row=10, column=3)


    



    def browse_folder(self):
        folder_path = filedialog.askdirectory()
        self.folder_path_entry.delete(0, tk.END)
        self.folder_path_entry.insert(0, folder_path)




    def read_input_values(self) -> Dict[str, Any]:
        """Reads input values from the GUI and returns them as a dictionary."""
        return {
            'camera_a': self.camera_a_var.get(),
            'camera_b': self.camera_b_var.get(),
            'camera_c': self.camera_c_var.get(),
            'camera_d': self.camera_d_var.get(),
            'angle': self.angle_entry.get().strip(),
            'plant_name': self.plant_name_entry.get().strip(),
            'seconds': self.seconds_entry.get().strip(),
            'folder_path': self.folder_path_entry.get().strip()
        }



    # def update_configuration(self, inputs):
    #     """Updates the configuration based on input values."""
    #     timestamp = datetime.now().strftime('%Y-%m-%d-%H%M')
    #     inputs['timestamp'] = inputs['plant_name'] + timestamp
    #     inputs['folder_with_date'] = f"{inputs['folder_path']}/{inputs['timestamp']}"




    #     for key, value in inputs.items():
    #         self.config.set_value(key, value)
    #     self.config.save_config()


    def update_configuration(self, inputs: Dict[str, Any]) -> None:
        """Updates the configuration based on input values with proper sanitization."""
        # Sanitize plant name
        plant_name = validate_plant_name(inputs['plant_name'])
        if not plant_name:
            plant_name = "unnamed"
        
        # Build the timestamp and full key with sanitized name
        ts = datetime.now().strftime('%Y-%m-%d-%H%M')
        full_ts = f"{plant_name}{ts}"
        inputs['timestamp'] = full_ts
        inputs['plant_name'] = plant_name  # Use sanitized name

        # Sanitize and validate folder path
        folder_path = sanitize_path(inputs['folder_path'])
        if not folder_path:
            raise ValueError("Invalid folder path")
        
        folder_with_date = os.path.join(folder_path, full_ts)
        inputs['folder_with_date'] = folder_with_date
        inputs['folder_path'] = folder_path  # Use sanitized path

        # Persist to Config
        for key, value in inputs.items():
            self.config.set_value(key, value)
        self.config.save_config()

    def transfer_configuration(self) -> None:
        """Transfers the configuration file to the Raspberry Pi."""
        try:
            # UnifiedNetworkManager doesn't need explicit connect/disconnect
            self.network_manager.transfer_file_to_remote('params.json', '/home/photopi/params.json')
            print("Success", "Configuration transferred successfully.")
        except Exception as e:
            print("Error", f"Failed to transfer configuration: {e}")
            
    def validate_inputs(self, inputs: Dict[str, Any]) -> bool:
        """Validates input fields with proper validation functions."""
        # Validate angle
        if not inputs.get('angle'):
            messagebox.showerror("Input Error", "Angle cannot be empty.")
            return False
        angle = validate_angle(inputs['angle'])
        if angle is None:
            messagebox.showerror("Input Error", "Angle must be between 0 and 360.")
            return False
        
        # Validate plant name
        if not inputs.get('plant_name'):
            messagebox.showerror("Input Error", "Plant name cannot be empty.")
            return False
        plant_name = validate_plant_name(inputs['plant_name'])
        if not plant_name:
            messagebox.showerror("Input Error", "Plant name contains invalid characters.")
            return False
        
        # Validate seconds (capture delay)
        if not inputs.get('seconds'):
            messagebox.showerror("Input Error", "Image capture delay cannot be empty.")
            return False
        seconds = validate_positive_integer(inputs['seconds'], max_value=300)  # Max 5 minutes
        if seconds is None:
            messagebox.showerror("Input Error", "Image capture delay must be a positive number (max 300 seconds).")
            return False
        
        # Validate folder path
        if not inputs.get('folder_path'):
            messagebox.showerror("Input Error", "Folder path cannot be empty.")
            return False
        if not os.path.exists(inputs['folder_path']):
            messagebox.showerror("Input Error", "Folder path does not exist.")
            return False
        
        return True




    def submit(self) -> None:
        """
        Reads input values, validates them, updates the configuration, and transfers the configuration file.
        """
        inputs = self.read_input_values()
        if not self.validate_inputs(inputs):
            return
        try:
            self.update_configuration(inputs)
            self.transfer_configuration()
        except ValueError as e:
            messagebox.showerror("Configuration Error", str(e))

    def disable_buttons(self):
        """
        Disables the Inspect and Start Imaging buttons to prevent multiple clicks.
        """
        self.inspect_button.config(state=tk.DISABLED)
        self.start_imaging_button.config(state=tk.DISABLED)

    def enable_buttons(self):
        """
        Enables the Inspect and Start Imaging buttons.
        """
        self.inspect_button.config(state=tk.NORMAL)
        self.start_imaging_button.config(state=tk.NORMAL)
   
    def perform_inspection(self):
        """
        Creates and uses the CameraSystem instance for inspection.
        """
        self.disable_buttons()
        def task():
            try:
                if hasattr(self, 'ssh_client'):
                    camera_system = CameraSystem(self.ssh_client, self.master, self.config)
                    camera_system.inspect()
                    logging.info("Inspection performed successfully.")
                    self.update_gui_from_thread("Inspection performed successfully.")
                else:
                    self.update_gui_from_thread("SSH client is not initialized.")
            except Exception as e:
                self.update_gui_from_thread(f"Error during inspection: {str(e)}")
            finally:
                self.enable_buttons()

        threading.Thread(target=task).start()

    def start_imaging(self):
        """
        Starts a background task to create and use the CameraSystem instance for imaging.f
        """
        self.disable_buttons()
        def task():
            try:
                if hasattr(self, 'ssh_client'):
                    camera_system = CameraSystem(self.ssh_client, self.master, self.config)
                    camera_system.imaging()
                    self.update_gui_from_thread("Imaging started successfully.")
                    
                else:
                    self.update_gui_from_thread("SSH client is not initialized.")
            except Exception as e:
                self.update_gui_from_thread(f"Error during imaging: {str(e)}")
            finally:
                self.enable_buttons()

        threading.Thread(target=task).start()

    def reboot_raspberry_pi(self):
        """
        Reboots the Raspberry Pi by sending a reboot command over SSH.
        """
        try:
            self.ssh_client.exec_command('sudo reboot')
            logging.info("Reboot command sent to Raspberry Pi.")
            self.ssh_client.close()
            threading.Thread(target=self.wait_for_pi_to_reboot).start()
        except Exception as e:
            logging.error(f"Failed to send reboot command: {e}")
            messagebox.showerror("Reboot Error", f"Failed to send reboot command: {e}")

    def wait_for_pi_to_reboot(self) -> None:
        """
        Waits for the Raspberry Pi to come back online after a reboot.
        """
        time.sleep(SSH_RECONNECT_DELAY)  # Wait before attempting to reconnect
        pi_hostname = self.config.get_value('pi_hostname')
        while True:
            try:
                self.ssh_client = paramiko.SSHClient()
                # Use secure host key policy
                self.ssh_client.set_missing_host_key_policy(SecureHostKeyPolicy())
                self.ssh_client.connect(pi_hostname, username=self.pi_username, password=self.pi_password)
                logging.info("Raspberry Pi is back online.")
                self.master.after(GUI_UPDATE_DELAY_MS, lambda: messagebox.showinfo("Reboot", "Raspberry Pi is back online."))
                break
            except Exception as e:
                logging.info("Waiting for Raspberry Pi to come back online...")
                time.sleep(SSH_RECONNECT_RETRY_INTERVAL)

    def reset(self):
        """
        Resets the application by rebooting the Raspberry Pi.
        """
        reboot_confirmation = messagebox.askyesno("Reboot Confirmation", "Are you sure you want to reboot the Raspberry Pi?")
        if reboot_confirmation:
            self.reboot_raspberry_pi()

def main():
    root = tk.Tk()
    app = InputGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()

