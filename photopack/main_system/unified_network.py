# PhotoPi/photopack/main_system/unified_network.py
"""
Unified network management for the PhotoPi system.
Consolidates SSH operations and provides connection pooling.
"""

import paramiko
import logging
import threading
import time
import os
import posixpath
from contextlib import contextmanager
from typing import Dict, Any, Optional, List, Tuple, Generator
from concurrent.futures import ThreadPoolExecutor, as_completed

from .security import SecureHostKeyPolicy
from .constants import IMAGE_FILE_EXTENSIONS
from .retry_utils import ssh_retry, file_transfer_retry, network_retry

logger = logging.getLogger(__name__)


class SSHConnectionPool:
    """
    Manages a pool of SSH connections for reuse.
    """
    
    def __init__(self, hostname: str, username: str, password: str, max_connections: int = 3):
        """
        Initialize the connection pool.
        
        Args:
            hostname: SSH server hostname
            username: SSH username
            password: SSH password
            max_connections: Maximum number of concurrent connections
        """
        self.hostname = hostname
        self.username = username
        self.password = password
        self.max_connections = max_connections
        
        self._pool: List[paramiko.SSHClient] = []
        self._pool_lock = threading.Lock()
        self._active_connections = 0
        
    def _create_connection(self) -> paramiko.SSHClient:
        """Create a new SSH connection."""
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(SecureHostKeyPolicy())
        client.connect(
            self.hostname, 
            username=self.username, 
            password=self.password,
            timeout=30
        )
        return client
    
    @contextmanager
    def get_connection(self) -> Generator[paramiko.SSHClient, None, None]:
        """
        Get an SSH connection from the pool.
        
        Yields:
            SSH client connection
        """
        client = None
        try:
            with self._pool_lock:
                if self._pool:
                    client = self._pool.pop()
                elif self._active_connections < self.max_connections:
                    client = self._create_connection()
                    self._active_connections += 1
                else:
                    # Wait for a connection to become available
                    pass
            
            if client is None:
                # Fall back to creating a temporary connection
                client = self._create_connection()
                temp_connection = True
            else:
                temp_connection = False
            
            # Test the connection
            try:
                client.exec_command('echo "test"', timeout=5)
            except Exception:
                # Connection is stale, create a new one
                try:
                    client.close()
                except:
                    pass
                client = self._create_connection()
            
            yield client
            
        finally:
            if client:
                if temp_connection:
                    try:
                        client.close()
                    except:
                        pass
                else:
                    # Return to pool
                    with self._pool_lock:
                        if len(self._pool) < self.max_connections:
                            self._pool.append(client)
                        else:
                            try:
                                client.close()
                            except:
                                pass
                            self._active_connections -= 1
    
    def close_all(self) -> None:
        """Close all connections in the pool."""
        with self._pool_lock:
            for client in self._pool:
                try:
                    client.close()
                except:
                    pass
            self._pool.clear()
            self._active_connections = 0


class UnifiedNetworkManager:
    """
    Unified network manager that consolidates all SSH operations with retry logic.
    """
    
    def __init__(self, hostname: str, username: str, password: str):
        """
        Initialize the network manager.
        
        Args:
            hostname: SSH server hostname
            username: SSH username  
            password: SSH password
        """
        self.hostname = hostname
        self.username = username
        self.password = password
        self.connection_pool = SSHConnectionPool(hostname, username, password)
        
    @ssh_retry
    def execute_command(self, command: str, timeout: int = 30) -> Tuple[str, str]:
        """
        Execute a command on the remote server with retry logic.
        
        Args:
            command: Command to execute
            timeout: Command timeout in seconds
            
        Returns:
            Tuple of (stdout, stderr)
        """
        with self.connection_pool.get_connection() as client:
            stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
            
            stdout_data = stdout.read().decode('utf-8')
            stderr_data = stderr.read().decode('utf-8')
            
            if stderr_data:
                logger.warning(f"Command stderr: {stderr_data.strip()}")
                
            return stdout_data, stderr_data
    
    @file_transfer_retry
    def transfer_file_to_remote(self, local_path: str, remote_path: str) -> None:
        """
        Transfer a file to the remote server with retry logic.
        
        Args:
            local_path: Local file path
            remote_path: Remote file path
        """
        with self.connection_pool.get_connection() as client:
            sftp = client.open_sftp()
            try:
                # Create remote directory if it doesn't exist
                remote_dir = posixpath.dirname(remote_path)
                try:
                    sftp.stat(remote_dir)
                except FileNotFoundError:
                    # Directory doesn't exist, create it
                    self.execute_command(f'mkdir -p "{remote_dir}"')
                
                sftp.put(local_path, remote_path)
                logger.info(f"Transferred {local_path} to {remote_path}")
            finally:
                sftp.close()
    
    @file_transfer_retry
    def transfer_files_from_remote(self, remote_path: str, local_path: str) -> None:
        """
        Transfer all image files from a remote directory to local directory with retry logic.
        
        Args:
            remote_path: Remote directory path
            local_path: Local directory path
        """
        # Ensure local directory exists
        os.makedirs(local_path, exist_ok=True)
        
        with self.connection_pool.get_connection() as client:
            sftp = client.open_sftp()
            try:
                file_list = sftp.listdir(remote_path)
                logger.info(f"Found {len(file_list)} files on remote: {file_list}")
                
                for filename in file_list:
                    if not filename.lower().endswith(IMAGE_FILE_EXTENSIONS):
                        continue
                    
                    remote_file = posixpath.join(remote_path, filename)
                    local_file = os.path.join(local_path, filename)
                    
                    logger.info(f"Transferring {remote_file} → {local_file}")
                    sftp.get(remote_file, local_file)
                
                logger.info("File transfer completed")
                
            finally:
                sftp.close()
    
    @file_transfer_retry
    def fetch_image_data(self, remote_path: str) -> List[Tuple[bytes, str]]:
        """
        Fetch raw image data from a remote directory with retry logic.
        
        Args:
            remote_path: Remote directory path
            
        Returns:
            List of tuples containing (image_data, filename)
        """
        raw_images = []
        
        with self.connection_pool.get_connection() as client:
            sftp = client.open_sftp()
            try:
                file_list = sftp.listdir(remote_path)
                logger.info(f"Fetching images from {remote_path}: {file_list}")
                
                for filename in file_list:
                    if not filename.lower().endswith(IMAGE_FILE_EXTENSIONS):
                        continue
                    
                    remote_file = posixpath.join(remote_path, filename)
                    try:
                        with sftp.open(remote_file, 'rb') as file_handle:
                            data = file_handle.read()
                            raw_images.append((data, filename))
                            logger.info(f"Fetched {filename} ({len(data)} bytes)")
                    except IOError as e:
                        logger.error(f"Failed to fetch {remote_file}: {e}")
                
            finally:
                sftp.close()
        
        return raw_images
    
    def create_remote_directory(self, remote_path: str) -> None:
        """
        Create a directory on the remote server.
        
        Args:
            remote_path: Remote directory path to create
        """
        self.execute_command(f'mkdir -p "{remote_path}"')
        logger.info(f"Created remote directory: {remote_path}")
    
    @network_retry
    def test_connection(self) -> bool:
        """
        Test the SSH connection with retry logic.
        
        Returns:
            True if connection is successful, False otherwise
        """
        try:
            stdout, stderr = self.execute_command('echo "connection_test"', timeout=10)
            return 'connection_test' in stdout
        except Exception as e:
            logger.error(f"Connection test failed: {e}")
            return False
    
    def close(self) -> None:
        """Close all network connections."""
        self.connection_pool.close_all()
        logger.info("All network connections closed")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()