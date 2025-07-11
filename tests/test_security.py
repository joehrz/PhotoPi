# PhotoPi/tests/test_security.py
"""
Unit tests for security module.
"""

import pytest
import os
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock
import paramiko

from photopack.main_system.security import (
    SecureHostKeyPolicy,
    sanitize_filename,
    sanitize_path,
    validate_plant_name,
    validate_angle,
    validate_positive_integer
)


class TestSecureHostKeyPolicy:
    """Test the SecureHostKeyPolicy class."""
    
    def test_init_with_default_known_hosts(self):
        """Test initialization with default known_hosts file."""
        policy = SecureHostKeyPolicy()
        expected_path = os.path.expanduser("~/.ssh/known_hosts")
        assert policy.known_hosts_file == expected_path
    
    def test_init_with_custom_known_hosts(self):
        """Test initialization with custom known_hosts file."""
        custom_path = "/tmp/test_known_hosts"
        policy = SecureHostKeyPolicy(custom_path)
        assert policy.known_hosts_file == custom_path
    
    @patch('os.path.exists')
    @patch('builtins.input')
    def test_missing_host_key_with_known_hosts_file(self, mock_input, mock_exists):
        """Test missing host key behavior when known_hosts file exists."""
        mock_exists.return_value = True
        mock_input.return_value = 'yes'
        
        policy = SecureHostKeyPolicy()
        client = Mock()
        client._host_keys = {}
        hostname = "test.example.com"
        key = Mock()
        key.get_fingerprint.return_value = Mock()
        key.get_fingerprint.return_value.hex.return_value = "abcd1234"
        key.get_name.return_value = "ssh-rsa"
        
        # Should not raise exception
        policy.missing_host_key(client, hostname, key)
        mock_input.assert_called_once()
    
    @patch('os.path.exists')
    @patch('builtins.input')
    def test_missing_host_key_user_refuses(self, mock_input, mock_exists):
        """Test missing host key behavior when user refuses connection."""
        mock_exists.return_value = True
        mock_input.return_value = 'no'
        
        policy = SecureHostKeyPolicy()
        client = Mock()
        hostname = "test.example.com"
        key = Mock()
        key.get_fingerprint.return_value = Mock()
        key.get_fingerprint.return_value.hex.return_value = "abcd1234"
        
        with pytest.raises(paramiko.SSHException):
            policy.missing_host_key(client, hostname, key)


class TestSanitizeFilename:
    """Test the sanitize_filename function."""
    
    def test_basic_filename(self):
        """Test sanitization of basic valid filename."""
        result = sanitize_filename("test_file.jpg")
        assert result == "test_file.jpg"
    
    def test_dangerous_characters(self):
        """Test removal of dangerous characters."""
        result = sanitize_filename("../../../etc/passwd")
        assert result == "___etc_passwd"
    
    def test_path_traversal_protection(self):
        """Test protection against path traversal."""
        result = sanitize_filename("../../bad/file.txt")
        assert result == "__bad_file.txt"
    
    def test_special_characters(self):
        """Test handling of special characters."""
        result = sanitize_filename("file<>:|?*.txt")
        assert result == "file_______.txt"
    
    def test_double_dots(self):
        """Test handling of double dots."""
        result = sanitize_filename("file..name.txt")
        assert result == "file_name.txt"
    
    def test_hidden_file_protection(self):
        """Test protection against hidden files."""
        result = sanitize_filename(".hidden_file")
        assert result == "_hidden_file"
    
    def test_length_limit(self):
        """Test filename length limiting."""
        long_name = "a" * 300 + ".txt"
        result = sanitize_filename(long_name, max_length=20)
        assert len(result) <= 20
        assert result.endswith(".txt")
    
    def test_empty_filename(self):
        """Test handling of empty filename."""
        result = sanitize_filename("")
        assert result == "unnamed"
    
    def test_only_underscores(self):
        """Test handling of filename that becomes only underscores."""
        result = sanitize_filename("!@#$%")
        assert result == "unnamed"


class TestSanitizePath:
    """Test the sanitize_path function."""
    
    def test_valid_path(self):
        """Test sanitization of valid path."""
        with tempfile.TemporaryDirectory() as temp_dir:
            result = sanitize_path(temp_dir)
            assert result == str(Path(temp_dir).resolve())
    
    def test_nonexistent_path(self):
        """Test handling of non-existent path."""
        result = sanitize_path("/definitely/does/not/exist")
        # Should still return the resolved path even if it doesn't exist
        assert result is not None
    
    def test_path_with_allowed_dirs(self):
        """Test path validation with allowed directories."""
        with tempfile.TemporaryDirectory() as temp_dir:
            subdir = os.path.join(temp_dir, "subdir")
            os.makedirs(subdir)
            
            result = sanitize_path(subdir, allowed_dirs=[temp_dir])
            assert result == str(Path(subdir).resolve())
    
    def test_path_outside_allowed_dirs(self):
        """Test path rejection when outside allowed directories."""
        with tempfile.TemporaryDirectory() as temp_dir:
            with tempfile.TemporaryDirectory() as other_dir:
                result = sanitize_path(other_dir, allowed_dirs=[temp_dir])
                assert result is None
    
    def test_invalid_path(self):
        """Test handling of invalid path."""
        # Test with path that contains null bytes (invalid on most systems)
        result = sanitize_path("invalid\x00path")
        assert result is None


class TestValidatePlantName:
    """Test the validate_plant_name function."""
    
    def test_valid_names(self):
        """Test validation of valid plant names."""
        valid_names = [
            "Tomato Plant",
            "Plant-123",
            "Test_Plant_01",
            "Simple",
            "A B C"
        ]
        
        for name in valid_names:
            result = validate_plant_name(name)
            assert result is not None
            assert isinstance(result, str)
    
    def test_invalid_characters(self):
        """Test handling of invalid characters."""
        result = validate_plant_name("Plant<script>alert('xss')</script>")
        assert result == "Plantalert"
    
    def test_length_limit(self):
        """Test plant name length limiting."""
        long_name = "a" * 200
        result = validate_plant_name(long_name, max_length=50)
        assert len(result) <= 50
    
    def test_empty_name(self):
        """Test handling of empty name."""
        assert validate_plant_name("") is None
        assert validate_plant_name("   ") is None
    
    def test_whitespace_handling(self):
        """Test proper whitespace handling."""
        result = validate_plant_name("  Plant   Name  ")
        assert result == "Plant Name"
    
    def test_multiple_spaces(self):
        """Test handling of multiple consecutive spaces."""
        result = validate_plant_name("Plant    Name")
        assert result == "Plant Name"


class TestValidateAngle:
    """Test the validate_angle function."""
    
    def test_valid_angles(self):
        """Test validation of valid angles."""
        valid_angles = ["0", "30", "90", "180", "270", "360"]
        
        for angle in valid_angles:
            result = validate_angle(angle)
            assert result is not None
            assert isinstance(result, int)
            assert 0 <= result <= 360
    
    def test_invalid_angles(self):
        """Test handling of invalid angles."""
        invalid_angles = ["-1", "361", "abc", "30.5", ""]
        
        for angle in invalid_angles:
            result = validate_angle(angle)
            assert result is None
    
    def test_boundary_values(self):
        """Test boundary value validation."""
        assert validate_angle("0") == 0
        assert validate_angle("360") == 360
        assert validate_angle("-1") is None
        assert validate_angle("361") is None


class TestValidatePositiveInteger:
    """Test the validate_positive_integer function."""
    
    def test_valid_integers(self):
        """Test validation of valid positive integers."""
        valid_values = ["1", "5", "100", "999"]
        
        for value in valid_values:
            result = validate_positive_integer(value)
            assert result is not None
            assert isinstance(result, int)
            assert result > 0
    
    def test_invalid_integers(self):
        """Test handling of invalid integers."""
        invalid_values = ["0", "-1", "abc", "1.5", ""]
        
        for value in invalid_values:
            result = validate_positive_integer(value)
            assert result is None
    
    def test_max_value_constraint(self):
        """Test maximum value constraint."""
        assert validate_positive_integer("50", max_value=100) == 50
        assert validate_positive_integer("150", max_value=100) is None
    
    def test_no_max_value(self):
        """Test validation without maximum value constraint."""
        result = validate_positive_integer("999999")
        assert result == 999999


if __name__ == "__main__":
    pytest.main([__file__])