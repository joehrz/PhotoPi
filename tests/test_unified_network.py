# PhotoPi/tests/test_unified_network.py
"""
Unit tests for unified network management module.
"""

import pytest
import paramiko
import threading
import time
from unittest.mock import Mock, patch, MagicMock, call
from contextlib import contextmanager

from photopack.main_system.unified_network import (
    SSHConnectionPool,
    UnifiedNetworkManager
)


class TestSSHConnectionPool:
    """Test the SSHConnectionPool class."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.hostname = "test.example.com"
        self.username = "testuser"
        self.password = "testpass"
    
    @patch('photopack.main_system.unified_network.paramiko.SSHClient')
    def test_create_connection(self, mock_ssh_client):
        """Test creation of new SSH connection."""
        mock_client = Mock()
        mock_ssh_client.return_value = mock_client
        
        pool = SSHConnectionPool(self.hostname, self.username, self.password)
        client = pool._create_connection()
        
        assert client == mock_client
        mock_client.set_missing_host_key_policy.assert_called_once()
        mock_client.connect.assert_called_once_with(
            self.hostname,
            username=self.username,
            password=self.password,
            timeout=30
        )
    
    @patch('photopack.main_system.unified_network.paramiko.SSHClient')
    def test_get_connection_new(self, mock_ssh_client):
        """Test getting connection when pool is empty."""
        mock_client = Mock()
        mock_ssh_client.return_value = mock_client
        mock_client.exec_command.return_value = (Mock(), Mock(), Mock())
        
        pool = SSHConnectionPool(self.hostname, self.username, self.password, max_connections=2)
        
        with pool.get_connection() as client:
            assert client == mock_client
            assert pool._active_connections == 1
    
    @patch('photopack.main_system.unified_network.paramiko.SSHClient')
    def test_get_connection_from_pool(self, mock_ssh_client):
        """Test getting connection from existing pool."""
        mock_client1 = Mock()
        mock_client2 = Mock()
        mock_ssh_client.side_effect = [mock_client1, mock_client2]
        
        # Set up mock exec_command for connection testing
        mock_client1.exec_command.return_value = (Mock(), Mock(), Mock())
        mock_client2.exec_command.return_value = (Mock(), Mock(), Mock())
        
        pool = SSHConnectionPool(self.hostname, self.username, self.password, max_connections=2)
        
        # Use first connection and return it to pool
        with pool.get_connection() as client:
            assert client == mock_client1
        
        # Second connection should reuse from pool
        with pool.get_connection() as client:
            assert client == mock_client1  # Reused from pool
    
    @patch('photopack.main_system.unified_network.paramiko.SSHClient')
    def test_stale_connection_replacement(self, mock_ssh_client):
        """Test replacement of stale connections."""
        mock_client1 = Mock()
        mock_client2 = Mock()
        mock_ssh_client.side_effect = [mock_client1, mock_client2]
        
        # First client fails connection test (stale)
        mock_client1.exec_command.side_effect = Exception("Connection lost")
        # Second client works
        mock_client2.exec_command.return_value = (Mock(), Mock(), Mock())
        
        pool = SSHConnectionPool(self.hostname, self.username, self.password)
        pool._pool.append(mock_client1)  # Simulate existing connection in pool
        
        with pool.get_connection() as client:
            assert client == mock_client2  # Should get new connection
            mock_client1.close.assert_called_once()  # Stale connection closed
    
    def test_close_all_connections(self):
        """Test closing all connections in pool."""
        mock_client1 = Mock()
        mock_client2 = Mock()
        
        pool = SSHConnectionPool(self.hostname, self.username, self.password)
        pool._pool = [mock_client1, mock_client2]
        pool._active_connections = 2
        
        pool.close_all()
        
        assert len(pool._pool) == 0
        assert pool._active_connections == 0
        mock_client1.close.assert_called_once()
        mock_client2.close.assert_called_once()
    
    def test_close_all_handles_exceptions(self):
        """Test that close_all handles exceptions gracefully."""
        mock_client = Mock()
        mock_client.close.side_effect = Exception("Close failed")
        
        pool = SSHConnectionPool(self.hostname, self.username, self.password)
        pool._pool = [mock_client]
        pool._active_connections = 1
        
        # Should not raise exception
        pool.close_all()
        
        assert len(pool._pool) == 0
        assert pool._active_connections == 0


class TestUnifiedNetworkManager:
    """Test the UnifiedNetworkManager class."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.hostname = "test.example.com"
        self.username = "testuser"
        self.password = "testpass"
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_init(self, mock_pool_class):
        """Test initialization of UnifiedNetworkManager."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        
        assert manager.hostname == self.hostname
        assert manager.username == self.username
        assert manager.password == self.password
        assert manager.connection_pool == mock_pool
        mock_pool_class.assert_called_once_with(self.hostname, self.username, self.password)
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_execute_command_success(self, mock_pool_class):
        """Test successful command execution."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        mock_client = Mock()
        mock_stdout = Mock()
        mock_stderr = Mock()
        mock_stdout.read.return_value = b"command output"
        mock_stderr.read.return_value = b""
        
        mock_client.exec_command.return_value = (Mock(), mock_stdout, mock_stderr)
        mock_pool.get_connection.return_value.__enter__.return_value = mock_client
        mock_pool.get_connection.return_value.__exit__.return_value = None
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        stdout, stderr = manager.execute_command("ls -la")
        
        assert stdout == "command output"
        assert stderr == ""
        mock_client.exec_command.assert_called_once_with("ls -la", timeout=30)
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_execute_command_with_stderr(self, mock_pool_class):
        """Test command execution with stderr output."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        mock_client = Mock()
        mock_stdout = Mock()
        mock_stderr = Mock()
        mock_stdout.read.return_value = b"command output"
        mock_stderr.read.return_value = b"warning message"
        
        mock_client.exec_command.return_value = (Mock(), mock_stdout, mock_stderr)
        mock_pool.get_connection.return_value.__enter__.return_value = mock_client
        mock_pool.get_connection.return_value.__exit__.return_value = None
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        
        with patch('photopack.main_system.unified_network.logger') as mock_logger:
            stdout, stderr = manager.execute_command("ls -la")
            
            assert stdout == "command output"
            assert stderr == "warning message"
            mock_logger.warning.assert_called_once_with("Command stderr: warning message")
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    @patch('photopack.main_system.unified_network.os.makedirs')
    def test_transfer_file_to_remote(self, mock_makedirs, mock_pool_class):
        """Test file transfer to remote server."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        mock_client = Mock()
        mock_sftp = Mock()
        mock_client.open_sftp.return_value = mock_sftp
        
        # Mock successful stat (directory exists)
        mock_sftp.stat.return_value = Mock()
        
        mock_pool.get_connection.return_value.__enter__.return_value = mock_client
        mock_pool.get_connection.return_value.__exit__.return_value = None
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        manager.transfer_file_to_remote("/local/file.txt", "/remote/file.txt")
        
        mock_sftp.put.assert_called_once_with("/local/file.txt", "/remote/file.txt")
        mock_sftp.close.assert_called_once()
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    @patch('photopack.main_system.unified_network.os.makedirs')
    def test_transfer_file_to_remote_create_dir(self, mock_makedirs, mock_pool_class):
        """Test file transfer with remote directory creation."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        mock_client = Mock()
        mock_sftp = Mock()
        mock_client.open_sftp.return_value = mock_sftp
        
        # Mock FileNotFoundError (directory doesn't exist)
        mock_sftp.stat.side_effect = FileNotFoundError()
        
        mock_pool.get_connection.return_value.__enter__.return_value = mock_client
        mock_pool.get_connection.return_value.__exit__.return_value = None
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        
        # Mock execute_command for directory creation
        manager.execute_command = Mock()
        
        manager.transfer_file_to_remote("/local/file.txt", "/remote/dir/file.txt")
        
        manager.execute_command.assert_called_once_with('mkdir -p "/remote/dir"')
        mock_sftp.put.assert_called_once_with("/local/file.txt", "/remote/dir/file.txt")
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    @patch('photopack.main_system.unified_network.os.makedirs')
    @patch('photopack.main_system.unified_network.os.path.join')
    @patch('photopack.main_system.unified_network.posixpath.join')
    def test_transfer_files_from_remote(self, mock_posix_join, mock_path_join, mock_makedirs, mock_pool_class):
        """Test transferring files from remote server."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        mock_client = Mock()
        mock_sftp = Mock()
        mock_client.open_sftp.return_value = mock_sftp
        
        # Mock file list with image files
        mock_sftp.listdir.return_value = ["image1.jpg", "image2.png", "readme.txt", "image3.jpeg"]
        
        # Mock path joining
        mock_posix_join.side_effect = lambda *args: "/".join(args)
        mock_path_join.side_effect = lambda *args: "\\".join(args) if len(args) > 1 else args[0]
        
        mock_pool.get_connection.return_value.__enter__.return_value = mock_client
        mock_pool.get_connection.return_value.__exit__.return_value = None
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        manager.transfer_files_from_remote("/remote/images", "/local/images")
        
        # Should only transfer image files
        expected_calls = [
            call("/remote/images/image1.jpg", "/local/images\\image1.jpg"),
            call("/remote/images/image2.png", "/local/images\\image2.png"),
            call("/remote/images/image3.jpeg", "/local/images\\image3.jpeg")
        ]
        mock_sftp.get.assert_has_calls(expected_calls, any_order=True)
        assert mock_sftp.get.call_count == 3  # Only 3 image files
        mock_makedirs.assert_called_once_with("/local/images", exist_ok=True)
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_fetch_image_data(self, mock_pool_class):
        """Test fetching raw image data."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        mock_client = Mock()
        mock_sftp = Mock()
        mock_client.open_sftp.return_value = mock_sftp
        
        mock_sftp.listdir.return_value = ["image1.jpg", "image2.png", "readme.txt"]
        
        # Mock file handles
        mock_file1 = Mock()
        mock_file1.read.return_value = b"image1_data"
        mock_file2 = Mock()
        mock_file2.read.return_value = b"image2_data"
        
        mock_sftp.open.side_effect = [
            mock_file1.__enter__.return_value,
            mock_file2.__enter__.return_value
        ]
        mock_file1.__enter__ = Mock(return_value=mock_file1)
        mock_file1.__exit__ = Mock(return_value=None)
        mock_file2.__enter__ = Mock(return_value=mock_file2)
        mock_file2.__exit__ = Mock(return_value=None)
        
        mock_pool.get_connection.return_value.__enter__.return_value = mock_client
        mock_pool.get_connection.return_value.__exit__.return_value = None
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        
        with patch('photopack.main_system.unified_network.posixpath.join', side_effect=lambda *args: "/".join(args)):
            raw_images = manager.fetch_image_data("/remote/images")
        
        assert len(raw_images) == 2  # Only image files
        assert (b"image1_data", "image1.jpg") in raw_images
        assert (b"image2_data", "image2.png") in raw_images
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_create_remote_directory(self, mock_pool_class):
        """Test creating remote directory."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        manager.execute_command = Mock()
        
        manager.create_remote_directory("/remote/new/directory")
        
        manager.execute_command.assert_called_once_with('mkdir -p "/remote/new/directory"')
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_test_connection_success(self, mock_pool_class):
        """Test successful connection test."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        manager.execute_command = Mock(return_value=("connection_test\n", ""))
        
        result = manager.test_connection()
        
        assert result is True
        manager.execute_command.assert_called_once_with('echo "connection_test"', timeout=10)
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_test_connection_failure(self, mock_pool_class):
        """Test failed connection test."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        manager.execute_command = Mock(side_effect=Exception("Connection failed"))
        
        with patch('photopack.main_system.unified_network.logger') as mock_logger:
            result = manager.test_connection()
            
            assert result is False
            mock_logger.error.assert_called_once()
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_close(self, mock_pool_class):
        """Test closing network manager."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        manager = UnifiedNetworkManager(self.hostname, self.username, self.password)
        manager.close()
        
        mock_pool.close_all.assert_called_once()
    
    @patch('photopack.main_system.unified_network.SSHConnectionPool')
    def test_context_manager(self, mock_pool_class):
        """Test using manager as context manager."""
        mock_pool = Mock()
        mock_pool_class.return_value = mock_pool
        
        with UnifiedNetworkManager(self.hostname, self.username, self.password) as manager:
            assert isinstance(manager, UnifiedNetworkManager)
        
        mock_pool.close_all.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__])