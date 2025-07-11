# PhotoPi/photopack/main_system/security.py
"""
Security utilities for the PhotoPi system.
Handles SSH security, input sanitization, and validation.
"""

import os
import re
import logging
import paramiko
from pathlib import Path
from typing import Optional, List

logger = logging.getLogger(__name__)


class SecureHostKeyPolicy(paramiko.MissingHostKeyPolicy):
    """
    A more secure SSH host key policy that uses a known_hosts file.
    Falls back to warning mode if the file doesn't exist.
    """
    
    def __init__(self, known_hosts_file: Optional[str] = None):
        """
        Initialize the policy with an optional known_hosts file path.
        
        Args:
            known_hosts_file: Path to known_hosts file. If None, uses ~/.ssh/known_hosts
        """
        super().__init__()
        if known_hosts_file is None:
            known_hosts_file = os.path.expanduser("~/.ssh/known_hosts")
        self.known_hosts_file = known_hosts_file
        self.known_hosts_exists = os.path.exists(known_hosts_file)
        
    def missing_host_key(self, client, hostname, key):
        """
        Called when the host key is not found in known_hosts.
        """
        # Check if auto-accept is enabled
        auto_accept = os.getenv('PHOTOPI_AUTO_ACCEPT_HOST_KEY', 'false').lower() == 'true'
        
        if self.known_hosts_exists:
            # If we have a known_hosts file but the key isn't in it, warn the user
            logger.warning(
                f"Server '{hostname}' not found in known_hosts file. "
                f"Key fingerprint: {key.get_fingerprint().hex()}"
            )
            
            if auto_accept:
                logger.info(f"Auto-accepting host key for {hostname} due to PHOTOPI_AUTO_ACCEPT_HOST_KEY setting")
            else:
                # Cross-platform timeout implementation for user input
                try:
                    import threading
                    import sys
                    from queue import Queue, Empty
                    
                    def get_user_input(prompt, result_queue):
                        """Get user input in a separate thread."""
                        try:
                            response = input(prompt)
                            result_queue.put(response)
                        except EOFError:
                            result_queue.put(None)
                    
                    # Create a queue for the result
                    result_queue = Queue()
                    
                    # Start input thread
                    input_thread = threading.Thread(
                        target=get_user_input,
                        args=(f"Do you want to continue connecting to {hostname}? (yes/no): ", result_queue)
                    )
                    input_thread.daemon = True
                    input_thread.start()
                    
                    # Wait for input with timeout (30 seconds)
                    try:
                        response = result_queue.get(timeout=30)
                        if response is None:
                            raise paramiko.SSHException(f"Host key verification failed for {hostname}")
                        
                        if response.lower() != 'yes':
                            raise paramiko.SSHException(f"Host key verification failed for {hostname}")
                            
                    except Empty:
                        logger.error(f"Host key verification timeout for {hostname}")
                        raise paramiko.SSHException(f"Host key verification timeout for {hostname}")
                        
                except (KeyboardInterrupt, EOFError):
                    logger.error(f"Host key verification cancelled for {hostname}")
                    raise paramiko.SSHException(f"Host key verification cancelled for {hostname}")
        else:
            # No known_hosts file, just warn
            if auto_accept:
                logger.info(f"Auto-accepting host key for {hostname} (no known_hosts file)")
            else:
                logger.warning(
                    f"No known_hosts file found. Connecting to '{hostname}' "
                    f"with key fingerprint: {key.get_fingerprint().hex()}"
                )
        
        # Add the key to known_hosts for future connections
        if client._host_keys is not None:
            client._host_keys.add(hostname, key.get_name(), key)
            if self.known_hosts_file:
                try:
                    # Create directory if it doesn't exist
                    os.makedirs(os.path.dirname(self.known_hosts_file), exist_ok=True)
                    client.save_host_keys(self.known_hosts_file)
                    logger.info(f"Successfully saved host key to {self.known_hosts_file}")
                except Exception as e:
                    logger.error(f"Failed to save host key to {self.known_hosts_file}: {e}")
            else:
                logger.warning("No known_hosts file path configured, host key will not be saved")


def sanitize_filename(filename: str, max_length: int = 255) -> str:
    """
    Sanitize a filename to prevent path traversal and other security issues.
    
    Args:
        filename: The filename to sanitize
        max_length: Maximum allowed length for the filename
        
    Returns:
        Sanitized filename safe for use in file operations
    """
    # Remove any path separators
    filename = os.path.basename(filename)
    
    # Remove or replace potentially dangerous characters
    # Allow only alphanumeric, dash, underscore, and dot
    filename = re.sub(r'[^a-zA-Z0-9\-_.]', '_', filename)
    
    # Prevent double dots
    filename = re.sub(r'\.{2,}', '_', filename)
    
    # Ensure it doesn't start with a dot (hidden file)
    if filename.startswith('.'):
        filename = '_' + filename[1:]
    
    # Limit length
    if len(filename) > max_length:
        name, ext = os.path.splitext(filename)
        # Keep extension if possible
        if len(ext) < max_length:
            name = name[:max_length - len(ext)]
            filename = name + ext
        else:
            filename = filename[:max_length]
    
    # Ensure we have a valid filename
    if not filename or filename == '_':
        filename = 'unnamed'
    
    return filename


def sanitize_path(path: str, allowed_dirs: Optional[List[str]] = None) -> Optional[str]:
    """
    Sanitize a file path to prevent directory traversal attacks.
    
    Args:
        path: The path to sanitize
        allowed_dirs: List of allowed base directories. If provided, the path must be within one of these.
        
    Returns:
        Sanitized absolute path, or None if the path is invalid
    """
    try:
        # Convert to Path object and resolve to absolute path
        clean_path = Path(path).resolve()
        
        # Check if path exists and is not a symlink (unless following symlinks is desired)
        if clean_path.exists() and clean_path.is_symlink():
            logger.warning(f"Path {path} is a symlink")
        
        # If allowed directories are specified, check if path is within them
        if allowed_dirs:
            is_allowed = False
            for allowed_dir in allowed_dirs:
                allowed_path = Path(allowed_dir).resolve()
                try:
                    # Check if clean_path is relative to allowed_path
                    clean_path.relative_to(allowed_path)
                    is_allowed = True
                    break
                except ValueError:
                    continue
            
            if not is_allowed:
                logger.error(f"Path {path} is not within allowed directories")
                return None
        
        return str(clean_path)
        
    except Exception as e:
        logger.error(f"Invalid path {path}: {e}")
        return None


def validate_plant_name(name: str, max_length: int = 100) -> Optional[str]:
    """
    Validate and sanitize a plant name.
    
    Args:
        name: The plant name to validate
        max_length: Maximum allowed length
        
    Returns:
        Sanitized plant name, or None if invalid
    """
    if not name or not name.strip():
        return None
    
    # Remove leading/trailing whitespace
    name = name.strip()
    
    # Allow letters, numbers, spaces, hyphens, and underscores
    name = re.sub(r'[^a-zA-Z0-9\s\-_]', '', name)
    
    # Replace multiple spaces with single space
    name = re.sub(r'\s+', ' ', name)
    
    # Limit length
    if len(name) > max_length:
        name = name[:max_length]
    
    return name if name else None


def validate_angle(angle_str: str) -> Optional[int]:
    """
    Validate an angle input string.
    
    Args:
        angle_str: String representation of angle
        
    Returns:
        Validated angle as integer, or None if invalid
    """
    try:
        angle = int(angle_str)
        if 0 <= angle <= 360:
            return angle
    except (ValueError, TypeError):
        pass
    return None


def validate_positive_integer(value_str: str, max_value: Optional[int] = None) -> Optional[int]:
    """
    Validate a positive integer input.
    
    Args:
        value_str: String representation of integer
        max_value: Optional maximum allowed value
        
    Returns:
        Validated integer, or None if invalid
    """
    try:
        value = int(value_str)
        if value > 0:
            if max_value is None or value <= max_value:
                return value
    except (ValueError, TypeError):
        pass
    return None