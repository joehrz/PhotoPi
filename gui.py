import tkinter as tk
from tkinter import filedialog
from config import Config
from network import NetworkManager

class InputGUI:
    def __init__(self, master):
        self.master = master
        # Initialize Config and NetworkManager here
        self.config = Config()
        self.network_manager = NetworkManager(self.config.get_value('pi_hostname'),
                                              'pi',  # Username
                                              'photopi')  # Password, consider securing this
        self.setup_gui()

    def setup_gui(self):
        # Setup GUI components as before
        pass

    # Other methods as before, but now using self.config and self.network_manager
    # For example, in submit() method, instead of directly working with JSON:
    # self.config.set_value('key', value) and then self.config.save_config()

if __name__ == "__main__":
    root = tk.Tk()
    app = InputGUI(root)
    root.mainloop()