import tkinter as tk
from tkinter import messagebox
import socket
import logging


# Create a module-specific logger
logger = logging.getLogger(__name__)

class CredentialsDialog(tk.Toplevel):
    """
    Dialog for entering username and password to connect to Raspberry Pi.
    """
    def __init__(self, parent):
        """
        Initializes the dialog window.

        Parameters:
            parent (tk.Tk): The parent window.
        """
        super().__init__(parent)
        self.title("Login to Raspberry Pi")
        self.geometry("300x120")
        self.resizable(False, False)

        # Username and password labels and entries
        tk.Label(self, text="Username:").grid(row=0, column=0)
        self.username_entry = tk.Entry(self)
        self.username_entry.grid(row=0, column=1, padx=10, pady=5)
        tk.Label(self, text="Password:").grid(row=1, column=0)
        self.password_entry = tk.Entry(self, show="*")
        self.password_entry.grid(row=1, column=1, padx=10, pady=5)

        # Connect and Cancel buttons
        connect_button = tk.Button(self, text="Connect", command=self.on_connect)
        connect_button.grid(row=2, column=0, padx=10, pady=10, sticky='e')
        cancel_button = tk.Button(self, text="Cancel", command=self.destroy)
        cancel_button.grid(row=2, column=1, padx=10, pady=10, sticky='w')

        self.protocol("WM_DELETE_WINDOW", self.destroy)  # Handle window close
        self.transient(parent)  # Set to be a transient window of the main window
        self.grab_set()  # Take over input focus
        self.wait_window()  # Block until this window is destroyed

    def on_connect(self):
        """
        Handles the connect button click event.
        """
        if not self.username_entry.get() or not self.password_entry.get():
            messagebox.showerror("Error", "Both username and password are required!", parent=self)
            return
        self.username = self.username_entry.get()
        self.password = self.password_entry.get()
        self.destroy()

class NetworkConfig:
    """
    Manages network configuration and credentials for connecting to Raspberry Pi.
    """
    def __init__(self, root):
        """
        Initializes the NetworkConfig object.

        Parameters:
            root (tk.Tk): The root window of the Tkinter application.
        """
        self.root = root
        self.username = None
        self.password = None
        self.hostname = None

    def prompt_credentials(self):
        """
        Prompts the user for credentials using the CredentialsDialog.
        """
        dialog = CredentialsDialog(self.root)
        if hasattr(dialog, 'username') and hasattr(dialog, 'password'):
            self.username = dialog.username
            self.password = dialog.password
        else:
            raise ValueError("Login cancelled or credentials not provided.")

    def discover_pi_hostname(self):
        """
        Discovers the hostname of the Raspberry Pi on the local network.
        """
        try:
            self.hostname = socket.gethostbyname('raspberrypi.local')
            messagebox.showinfo("Network Info", f"Raspberry Pi found at IP: {self.hostname}", parent=self.root)
            logger.info(f"Raspberry Pi found at IP: {self.hostname}")
        except socket.error:
            messagebox.showerror("Network Error", "Failed to locate Raspberry Pi on the network.", parent=self.root)
            logger.error("Failed to locate Raspberry Pi on the network.")

    def get_credentials(self):
        """
        Returns the stored credentials.

        Returns:
            tuple: A tuple containing the username, password, and hostname.
        """
        return self.username, self.password, self.hostname