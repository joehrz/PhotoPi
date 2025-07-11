# PhotoPi/photopack/main_system/config_validation.py
"""
Configuration schema validation for the PhotoPi system.
Uses pydantic for robust validation of configuration files.
"""

import os
import json
import yaml
from pathlib import Path
from typing import Dict, Any, Optional, Union
from pydantic import BaseModel, Field, validator, ValidationError
import logging

logger = logging.getLogger(__name__)


class CameraSettings(BaseModel):
    """Schema for camera configuration settings."""
    camera_a: int = Field(ge=0, le=1, description="Camera A enabled (0 or 1)")
    camera_b: int = Field(ge=0, le=1, description="Camera B enabled (0 or 1)")
    camera_c: int = Field(ge=0, le=1, description="Camera C enabled (0 or 1)")
    camera_d: int = Field(ge=0, le=1, description="Camera D enabled (0 or 1)")
    
    @validator('*', pre=True)
    def convert_strings_to_int(cls, v):
        """Convert string values to integers for camera settings."""
        if isinstance(v, str) and v.isdigit():
            return int(v)
        return v


class CaptureSettings(BaseModel):
    """Schema for image capture settings."""
    plant_name: str = Field(min_length=1, max_length=100, description="Plant name for identification")
    angle: int = Field(ge=1, le=360, description="Rotation angle between captures")
    seconds: int = Field(ge=1, le=300, description="Delay between captures in seconds")
    folder_path: str = Field(min_length=1, description="Base folder path for saving images")
    
    @validator('plant_name')
    def validate_plant_name(cls, v):
        """Validate plant name contains only safe characters."""
        if not all(c.isalnum() or c in ' -_' for c in v):
            raise ValueError("Plant name can only contain letters, numbers, spaces, hyphens, and underscores")
        return v.strip()
    
    @validator('folder_path')
    def validate_folder_path(cls, v):
        """Validate folder path exists and is accessible."""
        path = Path(v)
        if not path.exists():
            raise ValueError(f"Folder path does not exist: {v}")
        if not path.is_dir():
            raise ValueError(f"Path is not a directory: {v}")
        if not os.access(str(path), os.W_OK):
            raise ValueError(f"No write permission for directory: {v}")
        return str(path.resolve())


class SystemSettings(BaseModel):
    """Schema for system-level settings."""
    pi_hostname: Optional[str] = Field(description="Raspberry Pi hostname")
    timestamp: Optional[str] = Field(description="Generated timestamp")
    folder_with_date: Optional[str] = Field(description="Generated folder path with timestamp")


class MainSystemConfig(BaseModel):
    """Complete schema for main system configuration (params.json)."""
    # Required settings
    plant_name: str = Field(min_length=1, max_length=100)
    angle: int = Field(ge=1, le=360)
    seconds: int = Field(ge=1, le=300)
    folder_path: str = Field(min_length=1)
    
    # Camera settings
    camera_a: int = Field(ge=0, le=1, default=0)
    camera_b: int = Field(ge=0, le=1, default=0)
    camera_c: int = Field(ge=0, le=1, default=0)
    camera_d: int = Field(ge=0, le=1, default=0)
    
    # Optional system settings
    pi_hostname: Optional[str] = None
    timestamp: Optional[str] = None
    folder_with_date: Optional[str] = None
    
    @validator('plant_name')
    def validate_plant_name(cls, v):
        """Validate plant name."""
        if not all(c.isalnum() or c in ' -_' for c in v):
            raise ValueError("Plant name can only contain letters, numbers, spaces, hyphens, and underscores")
        return v.strip()
    
    @validator('folder_path')
    def validate_folder_path(cls, v):
        """Validate folder path."""
        path = Path(v)
        if not path.exists():
            raise ValueError(f"Folder path does not exist: {v}")
        return str(path.resolve())
    
    @validator('camera_a', 'camera_b', 'camera_c', 'camera_d', pre=True)
    def convert_camera_settings(cls, v):
        """Convert string values to integers for camera settings."""
        if isinstance(v, str) and v.isdigit():
            return int(v)
        return v
    
    def at_least_one_camera_enabled(self):
        """Validate that at least one camera is enabled."""
        if not any([self.camera_a, self.camera_b, self.camera_c, self.camera_d]):
            raise ValueError("At least one camera must be enabled")
        return self


class SiftExtractionConfig(BaseModel):
    """Schema for SIFT feature extraction settings."""
    max_image_size: int = Field(ge=100, le=10000, default=3200)
    max_num_features: int = Field(ge=100, le=50000, default=8192)
    estimate_affine_shape: bool = Field(default=True)
    domain_size_pooling: bool = Field(default=True)


class ExhaustiveMatcherConfig(BaseModel):
    """Schema for exhaustive matcher settings."""
    guided_matching: bool = Field(default=True)
    block_size: int = Field(ge=1, le=1000, default=75)
    loop_detection_num_images: int = Field(ge=1, le=1000, default=30)


class MapperConfig(BaseModel):
    """Schema for COLMAP mapper settings."""
    ba_local_max_num_iterations: int = Field(ge=1, le=1000, default=30)
    ba_global_max_num_iterations: int = Field(ge=1, le=1000, default=75)
    ba_global_images_ratio: float = Field(ge=1.0, le=10.0, default=1.1)
    ba_global_points_ratio: float = Field(ge=1.0, le=10.0, default=1.1)
    ba_global_max_refinements: int = Field(ge=1, le=100, default=5)
    ba_local_max_refinements: int = Field(ge=1, le=100, default=2)


class PatchMatchStereoConfig(BaseModel):
    """Schema for patch match stereo settings."""
    max_image_size: int = Field(ge=100, le=10000, default=3200)
    window_radius: int = Field(ge=1, le=50, default=5)
    window_step: int = Field(ge=1, le=10, default=1)
    num_samples: int = Field(ge=1, le=100, default=15)
    num_iterations: int = Field(ge=1, le=100, default=5)
    geom_consistency: bool = Field(default=False)


class StereoFusionConfig(BaseModel):
    """Schema for stereo fusion settings."""
    check_num_images: int = Field(ge=1, le=1000, default=50)
    max_image_size: int = Field(ge=100, le=10000, default=3200)


class RemoteServerConfig(BaseModel):
    """Complete schema for remote server configuration (config.yaml)."""
    sift_extraction: SiftExtractionConfig = Field(default_factory=SiftExtractionConfig)
    exhaustive_matcher: ExhaustiveMatcherConfig = Field(default_factory=ExhaustiveMatcherConfig)
    mapper: MapperConfig = Field(default_factory=MapperConfig)
    patch_match_stereo: PatchMatchStereoConfig = Field(default_factory=PatchMatchStereoConfig)
    stereo_fusion: StereoFusionConfig = Field(default_factory=StereoFusionConfig)


class ConfigValidator:
    """
    Configuration validator that handles loading and validating config files.
    """
    
    @staticmethod
    def validate_main_config(config_data: Dict[str, Any]) -> MainSystemConfig:
        """
        Validate main system configuration.
        
        Args:
            config_data: Configuration dictionary
            
        Returns:
            Validated configuration object
            
        Raises:
            ValidationError: If validation fails
        """
        try:
            config = MainSystemConfig(**config_data)
            config.at_least_one_camera_enabled()
            return config
        except ValidationError as e:
            logger.error(f"Main config validation failed: {e}")
            raise
    
    @staticmethod
    def validate_remote_config(config_data: Dict[str, Any]) -> RemoteServerConfig:
        """
        Validate remote server configuration.
        
        Args:
            config_data: Configuration dictionary
            
        Returns:
            Validated configuration object
            
        Raises:
            ValidationError: If validation fails
        """
        try:
            return RemoteServerConfig(**config_data)
        except ValidationError as e:
            logger.error(f"Remote config validation failed: {e}")
            raise
    
    @staticmethod
    def load_and_validate_json(file_path: Union[str, Path], schema_class: type) -> BaseModel:
        """
        Load and validate a JSON configuration file.
        
        Args:
            file_path: Path to JSON file
            schema_class: Pydantic model class for validation
            
        Returns:
            Validated configuration object
        """
        try:
            with open(file_path, 'r') as f:
                config_data = json.load(f)
            
            if schema_class == MainSystemConfig:
                return ConfigValidator.validate_main_config(config_data)
            else:
                return schema_class(**config_data)
                
        except FileNotFoundError:
            logger.error(f"Configuration file not found: {file_path}")
            raise
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON in {file_path}: {e}")
            raise
        except ValidationError as e:
            logger.error(f"Configuration validation failed for {file_path}: {e}")
            raise
    
    @staticmethod
    def load_and_validate_yaml(file_path: Union[str, Path], schema_class: type) -> BaseModel:
        """
        Load and validate a YAML configuration file.
        
        Args:
            file_path: Path to YAML file
            schema_class: Pydantic model class for validation
            
        Returns:
            Validated configuration object
        """
        try:
            with open(file_path, 'r') as f:
                config_data = yaml.safe_load(f)
            
            if schema_class == RemoteServerConfig:
                return ConfigValidator.validate_remote_config(config_data)
            else:
                return schema_class(**config_data)
                
        except FileNotFoundError:
            logger.error(f"Configuration file not found: {file_path}")
            raise
        except yaml.YAMLError as e:
            logger.error(f"Invalid YAML in {file_path}: {e}")
            raise
        except ValidationError as e:
            logger.error(f"Configuration validation failed for {file_path}: {e}")
            raise
    
    @staticmethod
    def create_default_main_config() -> Dict[str, Any]:
        """Create a default main system configuration."""
        import platform
        import tempfile
        
        # Create platform-appropriate default folder path
        if platform.system() == "Windows":
            # Use user's temp directory on Windows
            default_folder = os.path.join(tempfile.gettempdir(), "photopi_images")
        else:
            # Use /tmp on Unix-like systems
            default_folder = "/tmp/photopi_images"
        
        # Ensure the default folder exists
        os.makedirs(default_folder, exist_ok=True)
        
        return {
            "plant_name": "sample_plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": default_folder,
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
    
    @staticmethod
    def create_default_remote_config() -> Dict[str, Any]:
        """Create a default remote server configuration."""
        return RemoteServerConfig().dict()


def validate_config_file(file_path: str, config_type: str = "auto") -> BaseModel:
    """
    Convenience function to validate a configuration file.
    
    Args:
        file_path: Path to configuration file
        config_type: Type of config ("main", "remote", or "auto" to detect from filename)
        
    Returns:
        Validated configuration object
    """
    path = Path(file_path)
    
    # Auto-detect config type from filename
    if config_type == "auto":
        if path.name == "params.json":
            config_type = "main"
        elif path.name == "config.yaml":
            config_type = "remote"
        else:
            # Guess from extension
            if path.suffix == ".json":
                config_type = "main"
            elif path.suffix in [".yaml", ".yml"]:
                config_type = "remote"
            else:
                raise ValueError(f"Cannot determine config type for {file_path}")
    
    # Validate based on type
    if config_type == "main":
        return ConfigValidator.load_and_validate_json(file_path, MainSystemConfig)
    elif config_type == "remote":
        return ConfigValidator.load_and_validate_yaml(file_path, RemoteServerConfig)
    else:
        raise ValueError(f"Unknown config type: {config_type}")


if __name__ == "__main__":
    # Example usage
    try:
        # Test with sample data
        sample_config = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        validated = ConfigValidator.validate_main_config(sample_config)
        print("Configuration validation successful!")
        print(f"Validated config: {validated.dict()}")
        
    except ValidationError as e:
        print(f"Validation failed: {e}")