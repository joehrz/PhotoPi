# PhotoPi/photopack/main_system/config.py

import json
import logging

# Create a module-specific logger
logger = logging.getLogger(__name__)

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
        self.data = self.load()

    def load(self):
        """
        Attempts to load the configuration from the specified JSON file, initializing
        with defaults if the file is absent or invalid.

        Returns:
            dict: The loaded configuration data.
        """
        try:
            with open(self.filepath, 'r') as f:
                data = json.load(f)
                if not data:
                    return self.initialize_defaults()
                return data
        except FileNotFoundError:
            logger.warning(f"Configuration file {self.filepath} not found. Initializing with default settings.")
            return self.initialize_defaults()
        except json.JSONDecodeError as e:
            logger.error(f"Error decoding JSON from {self.filepath}: {e}")
            return self.initialize_defaults()

    def initialize_defaults(self):
        """
        Initialize default configuration settings.

        Returns:
            dict: A dictionary containing default settings.
        """
        defaults = {
            "camera_a": 0,
            "camera_b": 0,
            "angle": 90,
            "seconds": 10,
            "plant_name": "Default Plant",
            "folder_path": "/path/to/default"
        }
        logger.info("Initialized with default settings.")
        return defaults

    def save_config(self):
        """
        Attempts to save the current configuration data to the JSON file.
        Catches and logs exceptions related to file writing.
        """
        try:
            with open(self.filepath, 'w') as f:
                json.dump(self.data, f, indent=4)
            logger.info(f"Configuration saved to {self.filepath}.")
        except IOError as e:
            logger.error(f"Failed to save configuration to {self.filepath}: {e}")

    def get_value(self, key):
        """
        Retrieves a value from the configuration data based on the given key.

        Parameters:
            key (str): The key for which the value should be retrieved.

        Returns:
            The value associated with the key, or None if the key is not found.
        """
        value = self.data.get(key, None)
        logger.debug(f"Retrieved value for key '{key}': {value}")
        return value

    def set_value(self, key, value):
        """
        Sets or updates a value in the configuration data.

        Parameters:
            key (str): The key for the value to be set.
            value: The value to be set for the given key.
        """
        self.data[key] = value
        logger.debug(f"Set value for key '{key}': {value}")