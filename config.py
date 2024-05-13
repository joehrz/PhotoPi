import json


class Config:
    """
    Manages configuration settings stored in a JSON file.

    Provides functionality to load, save, and modify settings.
    """

    def __init__(self, filepath='params.json'):
        """
        Initializes the Config object by loading data from a specified JSON file.

        Parameters:
            filepath (str): The file path to the JSON configuration file. Defaults to 'params.json'.
        """
        self.filepath = filepath
        self.data = self.load_config()

    def load_config(self):
        """
        Attempts to load the configuration from the specified JSON file, initializing
        with defaults if the file is absent.
        """
        try:
            with open(self.filepath, 'r') as f:
                data = json.load(f)
                if not data:
                    return self.initialize_defaults()
                return data
        except FileNotFoundError:
            return self.initialize_defaults()

    def initialize_defaults(self):
        """
        Initialize default configuration settings.

        Returns:
            dict: A dictionary containing default settings.
        """
        # Example defaults; customize as necessary
        return {
            "camera_a": 0,
            "camera_b": 0,
            "angle": 90,
            "seconds": 10,
            "plant_name": "Default Plant",
            "folder_path": "/path/to/default"
        }
    
    def save_config(self):
        """
        Attempts to save the current configuration data to the JSON file.
        Catches and logs exceptions related to file writing.
        """
        try:
            with open(self.filepath, 'w') as f:
                json.dump(self.data, f, indent=4)
        except IOError as e:
            # Log the error or inform the user
            print(f"Failed to save configuration: {e}")

    def get_value(self, key):
        """
        Retrieves a value from the configuration data based on the given key.

        Parameters:
            key (str): The key for which the value should be retrieved.

        Returns:
            The value associated with the key, or None if the key is not found.
        """
        return self.data.get(key, None)

    def set_value(self, key, value):
        """
        Sets or updates a value in the configuration data.

        Parameters:
            key (str): The key for the value to be set.
            value: The value to be set for the given key.
        """
        self.data[key] = value