import paramiko
import logging

# Configure logging
logger = logging.getLogger(__name__)

class NetworkManager:
    """
    Manages the SSH and SFTP connections to a remote host.
    """

    def __init__(self, hostname, username, password):
        """
        Initializes the NetworkManager with hostname, username, and password.

        Parameters:
            hostname (str): The hostname of the remote machine.
            username (str): The SSH username.
            password (str): The SSH password.
        """
        self.hostname = hostname
        self.username = username
        self.password = password
        self.client = None
        self.sftp = None

    def connect(self):
        """
        Establishes the SSH and SFTP connections.
        """
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.client.connect(self.hostname, username=self.username, password=self.password)
            self.sftp = self.client.open_sftp()
            logging.info(f"Connected to {self.hostname} via SSH.")
        except paramiko.AuthenticationException:
            logging.error("Authentication failed, please check your credentials.")
            raise
        except paramiko.SSHException as e:
            logging.error(f"SSH connection error: {e}")
            raise

    def disconnect(self):
        """
        Closes the SSH and SFTP connections.
        """
        if self.sftp:
            self.sftp.close()
            logging.info("SFTP connection closed.")
        if self.client:
            self.client.close()
            logging.info("SSH connection closed.")

    def execute_command(self, command):
        """
        Executes a command on the remote machine.

        Parameters:
            command (str): The command to execute.

        Returns:
            str: The output of the command.

        Raises:
            Exception: If the SSH client is not connected.
        """
        if not self.client:
            raise Exception("SSH Client not connected.")
        stdin, stdout, stderr = self.client.exec_command(command)
        logging.info(f"Executed command: {command}")
        return stdout.read().decode()

    def transfer_file(self, local_path, remote_path):
        """
        Transfers a file to the remote machine.

        Parameters:
            local_path (str): The local file path.
            remote_path (str): The remote file path.

        Raises:
            Exception: If the SFTP client is not connected.
        """
        if not self.sftp:
            raise Exception("SFTP Client not connected.")
        self.sftp.put(local_path, remote_path)
        logging.info(f"Transferred file from {local_path} to {remote_path}")