# PhotoPi/photopack/main_system/config.py

import json
import logging
from typing import Dict, Any, Optional
from pydantic import ValidationError

from .config_validation import ConfigValidator, MainSystemConfig

# Create a module-specific logger
logger = logging.getLogger(__name__)

class Config:
    """
    Manages configuration settings stored in a JSON file with validation.

    Provides functionality to load, save, and modify settings with schema validation.
    """

    def __init__(self, filepath: str = 'params.json') -> None:
        """
        Initializes the Config object by loading data from a specified JSON file.

        Parameters:
            filepath (str): The file path to the JSON configuration file. Defaults to 'params.json'.
        """
        self.filepath = filepath
        self.data = self.load()
        self._validated_config: Optional[MainSystemConfig] = None

    def load(self) -> Dict[str, Any]:
        """
        Attempts to load and validate the configuration from the specified JSON file,
        initializing with defaults if the file is absent or invalid.

        Returns:
            dict: The loaded configuration data.
        """
        try:
            with open(self.filepath, 'r') as f:
                data = json.load(f)
                if not data:
                    return self.initialize_defaults()
                
                # Validate the loaded configuration
                try:
                    self._validated_config = ConfigValidator.validate_main_config(data)
                    logger.info(f"Configuration loaded and validated from {self.filepath}")
                    return data
                except ValidationError as e:
                    logger.error(f"Configuration validation failed: {e}")
                    logger.warning("Using default configuration due to validation errors")
                    return self.initialize_defaults()
                    
        except FileNotFoundError:
            logger.warning(f"Configuration file {self.filepath} not found. Initializing with default settings.")
            return self.initialize_defaults()
        except json.JSONDecodeError as e:
            logger.error(f"Error decoding JSON from {self.filepath}: {e}")
            return self.initialize_defaults()

    def initialize_defaults(self) -> Dict[str, Any]:
        """
        Initialize default configuration settings.

        Returns:
            dict: A dictionary containing default settings.
        """
        defaults = ConfigValidator.create_default_main_config()
        
        # Validate the defaults
        try:
            self._validated_config = ConfigValidator.validate_main_config(defaults)
            logger.info("Initialized with validated default settings.")
        except ValidationError as e:
            logger.error(f"Default configuration validation failed: {e}")
            # This should never happen, but fallback to minimal config
            defaults = {
                "camera_a": 1,
                "camera_b": 0, 
                "camera_c": 0,
                "camera_d": 0,
                "angle": 30,
                "seconds": 5,
                "plant_name": "Default Plant",
                "folder_path": "/tmp"
            }
        
        return defaults

    def save_config(self) -> None:
        """
        Attempts to save the current configuration data to the JSON file after validation.
        Catches and logs exceptions related to file writing.
        """
        try:
            # Validate before saving
            self._validated_config = ConfigValidator.validate_main_config(self.data)
            
            with open(self.filepath, 'w') as f:
                json.dump(self.data, f, indent=4)
            logger.info(f"Configuration validated and saved to {self.filepath}.")
            
        except ValidationError as e:
            logger.error(f"Configuration validation failed before saving: {e}")
            raise ValueError(f"Cannot save invalid configuration: {e}")
        except IOError as e:
            logger.error(f"Failed to save configuration to {self.filepath}: {e}")
            raise

    def get_value(self, key: str) -> Any:
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

    def set_value(self, key: str, value: Any) -> None:
        """
        Sets or updates a value in the configuration data.

        Parameters:
            key (str): The key for the value to be set.
            value: The value to be set for the given key.
        """
        self.data[key] = value
        logger.debug(f"Set value for key '{key}': {value}")
    
    def validate_current_config(self) -> bool:
        """
        Validate the current configuration without saving.
        
        Returns:
            bool: True if configuration is valid, False otherwise.
        """
        try:
            ConfigValidator.validate_main_config(self.data)
            return True
        except ValidationError as e:
            logger.error(f"Configuration validation failed: {e}")
            return False
    
    def get_validated_config(self) -> Optional[MainSystemConfig]:
        """
        Get the validated configuration object.
        
        Returns:
            MainSystemConfig object if validation successful, None otherwise.
        """
        if self._validated_config is None:
            try:
                self._validated_config = ConfigValidator.validate_main_config(self.data)
            except ValidationError:
                return None
        return self._validated_config