import paramiko

class NetworkManager:
    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.client = None
        self.sftp = None

    def connect(self):
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            self.client.connect(self.hostname, username=self.username, password=self.password)
            self.sftp = self.client.open_sftp()
        except paramiko.AuthenticationException:
            raise Exception("Authentication failed, please check your credentials.")
        except paramiko.SSHException as e:
            raise Exception(f"SSH connection error: {e}")

    def disconnect(self):
        if self.sftp:
            self.sftp.close()
        if self.client:
            self.client.close()

    def execute_command(self, command):
        if not self.client:
            raise Exception("SSH Client not connected.")
        stdin, stdout, stderr = self.client.exec_command(command)
        return stdout.read()

    def transfer_file(self, local_path, remote_path):
        if not self.sftp:
            raise Exception("SFTP Client not connected.")
        self.sftp.put(local_path, remote_path)