# PhotoPi/tests/test_config_validation.py
"""
Unit tests for configuration validation module.
"""

import pytest
import tempfile
import json
import yaml
import os
from pathlib import Path
from pydantic import ValidationError

from photopack.main_system.config_validation import (
    MainSystemConfig,
    RemoteServerConfig,
    ConfigValidator,
    validate_config_file
)


class TestMainSystemConfig:
    """Test the MainSystemConfig schema."""
    
    def test_valid_config(self):
        """Test validation of valid configuration."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        config = MainSystemConfig(**config_data)
        assert config.plant_name == "Test Plant"
        assert config.angle == 30
        assert config.seconds == 5
        assert config.camera_a == 1
    
    def test_string_to_int_conversion(self):
        """Test automatic conversion of string values to integers."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": "30",  # String that should be converted
            "seconds": "5",  # String that should be converted
            "folder_path": "/tmp",
            "camera_a": "1",  # String that should be converted
            "camera_b": "0",
            "camera_c": "0",
            "camera_d": "0"
        }
        
        config = MainSystemConfig(**config_data)
        assert config.angle == 30
        assert config.seconds == 5
        assert config.camera_a == 1
    
    def test_invalid_plant_name(self):
        """Test validation of invalid plant names."""
        config_data = {
            "plant_name": "Plant<script>",  # Invalid characters
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with pytest.raises(ValidationError):
            MainSystemConfig(**config_data)
    
    def test_invalid_angle_range(self):
        """Test validation of angle range."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 400,  # Out of range
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with pytest.raises(ValidationError):
            MainSystemConfig(**config_data)
    
    def test_invalid_seconds_range(self):
        """Test validation of seconds range."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 500,  # Out of range
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with pytest.raises(ValidationError):
            MainSystemConfig(**config_data)
    
    def test_nonexistent_folder_path(self):
        """Test validation of non-existent folder path."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/definitely/does/not/exist",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with pytest.raises(ValidationError):
            MainSystemConfig(**config_data)
    
    def test_at_least_one_camera_enabled(self):
        """Test validation that at least one camera is enabled."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 0,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        config = MainSystemConfig(**config_data)
        with pytest.raises(ValueError):
            config.at_least_one_camera_enabled()
    
    def test_optional_fields(self):
        """Test optional fields are handled correctly."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0,
            "pi_hostname": "raspberrypi.local",
            "timestamp": "2024-01-01-1200"
        }
        
        config = MainSystemConfig(**config_data)
        assert config.pi_hostname == "raspberrypi.local"
        assert config.timestamp == "2024-01-01-1200"


class TestRemoteServerConfig:
    """Test the RemoteServerConfig schema."""
    
    def test_default_config(self):
        """Test default configuration values."""
        config = RemoteServerConfig()
        
        assert config.sift_extraction.max_image_size == 3200
        assert config.sift_extraction.max_num_features == 8192
        assert config.exhaustive_matcher.block_size == 75
        assert config.mapper.ba_local_max_num_iterations == 30
    
    def test_custom_config(self):
        """Test custom configuration values."""
        config_data = {
            "sift_extraction": {
                "max_image_size": 1600,
                "max_num_features": 4096
            },
            "mapper": {
                "ba_local_max_num_iterations": 20
            }
        }
        
        config = RemoteServerConfig(**config_data)
        assert config.sift_extraction.max_image_size == 1600
        assert config.sift_extraction.max_num_features == 4096
        assert config.mapper.ba_local_max_num_iterations == 20
        # Default values should still be used for unspecified fields
        assert config.exhaustive_matcher.block_size == 75
    
    def test_invalid_values(self):
        """Test validation of invalid values."""
        config_data = {
            "sift_extraction": {
                "max_image_size": -100,  # Invalid negative value
                "max_num_features": 4096
            }
        }
        
        with pytest.raises(ValidationError):
            RemoteServerConfig(**config_data)


class TestConfigValidator:
    """Test the ConfigValidator class."""
    
    def test_validate_main_config_success(self):
        """Test successful main config validation."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        config = ConfigValidator.validate_main_config(config_data)
        assert isinstance(config, MainSystemConfig)
        assert config.plant_name == "Test Plant"
    
    def test_validate_main_config_failure(self):
        """Test main config validation failure."""
        config_data = {
            "plant_name": "",  # Invalid empty name
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 0,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with pytest.raises(ValidationError):
            ConfigValidator.validate_main_config(config_data)
    
    def test_validate_remote_config_success(self):
        """Test successful remote config validation."""
        config_data = {
            "sift_extraction": {
                "max_image_size": 1600
            }
        }
        
        config = ConfigValidator.validate_remote_config(config_data)
        assert isinstance(config, RemoteServerConfig)
        assert config.sift_extraction.max_image_size == 1600
    
    def test_load_and_validate_json(self):
        """Test loading and validating JSON configuration file."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(config_data, f)
            temp_file = f.name
        
        try:
            config = ConfigValidator.load_and_validate_json(temp_file, MainSystemConfig)
            assert isinstance(config, MainSystemConfig)
            assert config.plant_name == "Test Plant"
        finally:
            os.unlink(temp_file)
    
    def test_load_and_validate_yaml(self):
        """Test loading and validating YAML configuration file."""
        config_data = {
            "sift_extraction": {
                "max_image_size": 1600
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_data, f)
            temp_file = f.name
        
        try:
            config = ConfigValidator.load_and_validate_yaml(temp_file, RemoteServerConfig)
            assert isinstance(config, RemoteServerConfig)
            assert config.sift_extraction.max_image_size == 1600
        finally:
            os.unlink(temp_file)
    
    def test_load_invalid_json(self):
        """Test loading invalid JSON file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            f.write("{ invalid json")
            temp_file = f.name
        
        try:
            with pytest.raises(json.JSONDecodeError):
                ConfigValidator.load_and_validate_json(temp_file, MainSystemConfig)
        finally:
            os.unlink(temp_file)
    
    def test_load_nonexistent_file(self):
        """Test loading non-existent file."""
        with pytest.raises(FileNotFoundError):
            ConfigValidator.load_and_validate_json("/does/not/exist.json", MainSystemConfig)
    
    def test_create_default_configs(self):
        """Test creation of default configurations."""
        main_config = ConfigValidator.create_default_main_config()
        assert isinstance(main_config, dict)
        assert "plant_name" in main_config
        assert "angle" in main_config
        
        remote_config = ConfigValidator.create_default_remote_config()
        assert isinstance(remote_config, dict)
        assert "sift_extraction" in remote_config


class TestValidateConfigFile:
    """Test the validate_config_file function."""
    
    def test_auto_detect_params_json(self):
        """Test auto-detection of params.json file type."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(config_data, f)
            temp_file = f.name
        
        # Rename to params.json for auto-detection
        params_file = f.name.replace('.json', '_params.json')
        os.rename(temp_file, params_file)
        
        try:
            config = validate_config_file(params_file, config_type="auto")
            assert isinstance(config, MainSystemConfig)
        finally:
            os.unlink(params_file)
    
    def test_explicit_config_type(self):
        """Test explicit config type specification."""
        config_data = {
            "plant_name": "Test Plant",
            "angle": 30,
            "seconds": 5,
            "folder_path": "/tmp",
            "camera_a": 1,
            "camera_b": 0,
            "camera_c": 0,
            "camera_d": 0
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(config_data, f)
            temp_file = f.name
        
        try:
            config = validate_config_file(temp_file, config_type="main")
            assert isinstance(config, MainSystemConfig)
        finally:
            os.unlink(temp_file)
    
    def test_unknown_config_type(self):
        """Test handling of unknown config type."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump({}, f)
            temp_file = f.name
        
        try:
            with pytest.raises(ValueError):
                validate_config_file(temp_file, config_type="unknown")
        finally:
            os.unlink(temp_file)


if __name__ == "__main__":
    pytest.main([__file__])