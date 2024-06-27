import tkinter as tk
from tkinter import filedialog
import time
import json
from datetime import datetime, timedelta
import paramiko
import threading
from tkinter import messagebox

from imagecapture import CameraSystem
from credentials import NetworkConfig
from config import Config
from network import NetworkManager


class InputGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("PhotoPI")
        self.master.geometry("500x500")
        self.config = Config()
        self.network_manager = None
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        if self.initialize_network_and_ssh():
            self.setup_gui_components()
        else:
            self.master.quit()  # Optionally close the app if setup fails

    def initialize_network_and_ssh(self):
        """ Initialize network settings and SSH client using dynamic credentials. """
        net_config = NetworkConfig(self.master)
        try:
            net_config.prompt_credentials()
            net_config.discover_pi_hostname()

            if net_config.hostname and net_config.username and net_config.password:
                self.network_manager = NetworkManager(net_config.hostname, net_config.username, net_config.password)
                self.config.set_value('pi_hostname', net_config.hostname)
                self.config.save_config()

                self.ssh_client.connect(net_config.hostname, username=net_config.username, password=net_config.password)
                print("Network and SSH connection established successfully.")
                return True
            else:
                raise ValueError("Hostname, username, or password not provided or incorrect.")
        except (paramiko.ssh_exception.NoValidConnectionsError, ValueError) as e:
            messagebox.showerror("Connection Error", f"Failed to establish network or SSH connection: {str(e)}")
            return False

    def update_gui_from_thread(self, message):
        """Safely update the GUI from a background thread."""
        def update_status():
            if "successfully" in message:
                messagebox.showinfo("Success", message)
            else:
                messagebox.showerror("Error", message)
        self.master.after(100, update_status)
        


    def validate_angle_entry(self, text):
        if text == "":
            return True
        try:
            value = int(text)
        except ValueError:
            return False
        return 0 <= value <= 360

    def validate_text_entry(self, text):

        return True
        
    def setup_gui_components(self):
        self.camera_label = tk.Label(self.master, text="Cameras:")
        self.camera_label.grid(row=0,column=1)

        self.camera_a_var = tk.IntVar(value=0)
        self.camera_a_checkbutton = tk.Checkbutton(self.master, text="Camera A", variable=self.camera_a_var)
        self.camera_a_checkbutton.grid(row=1,column=1)

        self.camera_b_var = tk.IntVar(value=0)
        self.camera_b_checkbutton = tk.Checkbutton(self.master, text="Camera B", variable=self.camera_b_var)
        self.camera_b_checkbutton.grid(row=2,column=1)

        self.camera_c_var = tk.IntVar(value=0)
        self.camera_c_checkbutton = tk.Checkbutton(self.master, text="Camera C", variable=self.camera_c_var)
        self.camera_c_checkbutton.grid(row=3,column=1)

        self.camera_d_var = tk.IntVar(value=0)
        self.camera_d_checkbutton = tk.Checkbutton(self.master, text="Camera D", variable=self.camera_d_var)
        self.camera_d_checkbutton.grid(row=4,column=1)

        acmd = (self.master.register(self.validate_angle_entry), "%P")
        tcmd = (self.master.register(self.validate_text_entry), "%P")
    
        self.angle_label = tk.Label(self.master, text="Angle:")
        self.angle_label.grid(row=5)
        self.angle_entry = tk.Entry(self.master, validate="key", validatecommand=acmd)
        self.angle_entry.grid(row=5, column=1)

        self.seconds_label = tk.Label(self.master, text="Image Capture Delay:")
        self.seconds_label.grid(row=6)
        self.seconds_entry = tk.Entry(self.master, validate="key", validatecommand=acmd)
        self.seconds_entry.grid(row=6, column=1)

        self.plant_name_label = tk.Label(self.master, text="Plant Name:")
        self.plant_name_label.grid(row=7)
        self.plant_name_entry = tk.Entry(self.master, validate="key", validatecommand=tcmd)
        self.plant_name_entry.grid(row=7, column=1)

        tk.Label(self.master, text="Folder Path:").grid(row=8)
        self.folder_path_entry = tk.Entry(self.master)
        self.folder_path_entry.grid(row=8, column=1)

        tk.Button(self.master, text="Browse...", command=self.browse_folder).grid(row=8, column=2)
        tk.Button(self.master, text="Submit", command=self.submit).grid(row=9, column=0)
        tk.Button(self.master, text="Inspect Images", command=self.perform_inspection).grid(row=10, column=0)
        tk.Button(self.master, text="Start Imaging", command=self.start_imaging).grid(row=10, column=1)
        tk.Button(self.master, text="Quit", command=self.master.quit).grid(row=10, column=2)


    def browse_folder(self):
        folder_path = filedialog.askdirectory()
        self.folder_path_entry.delete(0, tk.END)
        self.folder_path_entry.insert(0, folder_path)




    def read_input_values(self):
        """Reads input values from the GUI and returns them as a dictionary."""
        return {
            'camera_a': self.camera_a_var.get(),
            'camera_b': self.camera_b_var.get(),
            'camera_c': self.camera_c_var.get(),
            'camera_d': self.camera_d_var.get(),
            'angle': self.angle_entry.get().strip(),  # Strip whitespace
            'plant_name': self.plant_name_entry.get().strip(),  # Strip whitespace
            'seconds': self.seconds_entry.get().strip(),  # Strip whitespace
            'folder_path': self.folder_path_entry.get().strip()  # Strip whitespace
        }

    def update_configuration(self, inputs):
        """Updates the configuration based on input values."""
        timestamp = datetime.now().strftime('%Y-%m-%d-%H%M')
        inputs['timestamp'] = inputs['plant_name'] + timestamp
        inputs['folder_with_date'] = f"{inputs['folder_path']}/{inputs['timestamp']}"

        for key, value in inputs.items():
            self.config.set_value(key, value)
        self.config.save_config()

    def transfer_configuration(self):
        """Transfers the configuration file to the Raspberry Pi."""
        try:
            self.network_manager.connect()
            self.network_manager.transfer_file('params.json', '/home/pi/params.json')
            print("Success", "Configuration transferred successfully.")
        except Exception as e:
            print("Error", f"Failed to transfer configuration: {e}")
        finally:
            self.network_manager.disconnect()
            
    def validate_inputs(self, inputs):
        """Validates input fields to ensure they are not empty."""
        for key, value in inputs.items():
            if key in ['angle', 'plant_name', 'seconds', 'folder_path'] and not value:  # Check specific fields for non-emptiness
                messagebox.showerror("Input Error", f"{key.replace('_', ' ').capitalize()} cannot be empty.")
                return False
        return True




    def submit(self):
        inputs = self.read_input_values()
        if not self.validate_inputs(inputs):  # Validate inputs before proceeding
            return  # Stop the submission process if validation fails
    
        self.update_configuration(inputs)
        self.transfer_configuration()
   
    def perform_inspection(self):
        """Create and use the CameraSystem instance for inspection."""
        if hasattr(self, 'ssh_client'):
            #camera_system = CameraSystem(self.ssh_client)
            camera_system = CameraSystem(self.ssh_client, root)
            camera_system.inspect()
            print("Inspection performed successfully.")
        else:
            print("SSH client is not initialized.")

    def start_imaging(self):
        def task():
            """Create and use the CameraSystem instance for imaging."""
            try:
                if hasattr(self, 'ssh_client'):
                    camera_system = CameraSystem(self.ssh_client, self.master)
                    camera_system.imaging()
                    self.update_gui_from_thread("Imaging started successfully.")
                else:
                    self.update_gui_from_thread("SSH client is not initialized.")
            except Exception as e:
                self.update_gui_from_thread(f"Error during imaging: {str(e)}")

        threading.Thread(target=task).start()



if __name__ == "__main__":
    root = tk.Tk()
    app = InputGUI(root)
    root.mainloop()