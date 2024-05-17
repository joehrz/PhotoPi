import tkinter as tk
from tkinter import filedialog
import time
import json
from datetime import datetime, timedelta
from inspection import CameraSystem
import paramiko

from credentials import NetworkConfig
from config import Config
from network import NetworkManager


class InputGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("PhotoPI")
        self.master.geometry("500x500")

        # Initialize Config and NetworkManager here
        self.config = Config()
        self.network_manager = None  # Initialize to None and set up later

        self.initialize_ssh_client()

        # Set up network configuration and manager
        if self.setup_network():
            self.setup_gui_components()
        else:
            self.master.quit()  # Optionally close the app if network setup fails

    def setup_network(self):
        """ Setup network configuration and initialize network manager. """
        net_config = NetworkConfig(self.master)  # Pass the root window to NetworkConfig
        try:
            net_config.prompt_credentials()
            net_config.discover_pi_hostname()

            if net_config.hostname:
                self.network_manager = NetworkManager(net_config.hostname, net_config.username, net_config.password)
                self.config.set_value('pi_hostname', net_config.hostname)
                self.config.save_config()
                return True
            else:
                print("Network Error", "Failed to locate the Raspberry Pi on the network.")
                return False
        except ValueError as e:
            print("Login Error", str(e))
            return False
        

    def initialize_ssh_client(self):
        """ Initialize the SSH client for CameraSystem. """
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        # Assuming credentials are correct and Raspberry Pi is accessible under 'raspberrypi.local'
        try:
            self.ssh_client.connect('raspberrypi.local', username='pi', password='raspberry')  # Replace with actual credentials
        except paramiko.ssh_exception.NoValidConnectionsError as e:
            print("Connection Error", "Could not connect to Raspberry Pi: " + str(e))
            self.master.quit()  # Close the application if connection fails

    def validate_angle_entry(self, text):
        if text == "": return True
        try:
            value = int(text)
        except ValueError: #oops, couldn't convert to int
            return False
        return 0 <= value <= 360

    def validate_text_entry(self, text):
        if text == "": return True
        try:
            value = str(text)
        except ValueError: #oops, couldn't convert to int
            return False
        
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
    
        self.angle_label = tk.Label(self.master, text="Angle:").grid(row=5)
        self.angle_entry = tk.Entry(self.master, validate = "key", validatecommand=acmd)
        self.angle_entry.grid(row=5, column=1)


        self.seconds_label = tk.Label(self.master, text="Image Capture Delay:").grid(row=6)
        self.seconds_label = tk.Entry(self.master, validate = "key", validatecommand=acmd)
        self.seconds_label.grid(row=6, column=1)


        self.plant_name_label = tk.Label(self.master, text="Plant Name:").grid(row=7)

        self.plant_name_entry = tk.Entry(self.master)
        self.plant_name_entry.grid(row=7, column=1)

        tk.Label(self.master, text="Folder Path:").grid(row=8)
        self.folder_path_entry = tk.Entry(self.master)
        self.folder_path_entry.grid(row=8, column=1)


        # Create buttons for folder selection and form submission
        tk.Button(self.master, text="Browse...", command=self.browse_folder).grid(row=8, column=2)
        tk.Button(self.master, text="Submit", command=self.submit).grid(row=9, column=0)

        # Create buttons for inspecting and starting imaging
        #tk.Button(self.master, text="Inspect Images", command=self.inspect).grid(row=10, column=0)
        tk.Button(self.master, text="Inspect Images", command=self.perform_inspection).grid(row=10, column=0)

        #tk.Button(master, text="Start Imaging", command=self.start_imaging).grid(row=10, column=1)

        tk.Button(self.master, text="Quit", command=self.master.quit).grid(row=10, column=2)

    def browse_folder(self):
        folder_path = filedialog.askdirectory()
        self.folder_path_entry.delete(0, tk.END)
        self.folder_path_entry.insert(0, folder_path)



    def submit(self):
        # Read input values
        camera_a = self.camera_a_var.get()
        camera_b = self.camera_b_var.get()
        camera_c = self.camera_c_var.get()
        camera_d = self.camera_d_var.get()
        angle = self.angle_entry.get()
        plant_name = self.plant_name_entry.get()
        seconds = self.seconds_label.get()  # Assuming seconds_entry is the correct reference
        folder_path = self.folder_path_entry.get()
        print("Folder path: {}".format(folder_path))

        today = datetime.now()
        the_time = '%04d-%02d-%02d-%02d%02d' % (today.year, today.month, today.day, today.hour, today.minute)
        timestamp = plant_name + ''  + the_time
        folderWithDate = folder_path + '/' + str(timestamp)

        # Update configuration using the instance of Config
        self.config.set_value('camera_a', camera_a)
        self.config.set_value('camera_b', camera_b)
        self.config.set_value('camera_c', camera_c)
        self.config.set_value('camera_d', camera_d)
        self.config.set_value('angle', angle)
        self.config.set_value('seconds', seconds)
        self.config.set_value('plant_name', plant_name)
        self.config.set_value('folder_path', folder_path)
        self.config.set_value('folderWithDate_path', folderWithDate)

        # Save the updated configuration
        self.config.save_config()

        # Transfer the updated config file to the Raspberry Pi
        try:
            self.network_manager.connect()  # Ensure we're connected
            self.network_manager.transfer_file('params.json', '/home/pi/params.json')
            print("Success", "Configuration transferred successfully.")
        except Exception as e:
            print("Error", f"Failed to transfer configuration: {e}")
        finally:
            self.network_manager.disconnect()  # Ensure cleanup of the connection
   
    def perform_inspection(self):
        """Create and use the CameraSystem instance for inspection."""
        if hasattr(self, 'ssh_client'):
            camera_system = CameraSystem(self.ssh_client)
            camera_system.inspect()
            print("Inspection performed successfully.")
        else:
            print("SSH client is not initialized.")
        

if __name__ == "__main__":
    root = tk.Tk()
    app = InputGUI(root)
    root.mainloop()