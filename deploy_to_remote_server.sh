#!/bin/bash

# deploy_to_remote_server.sh
# This script deploys the 3D reconstruction code to a remote server.

# -----------------------
# Load Environment Variables
# -----------------------
if [ -f .env ]; then
    export $(grep -v '^#' .env | xargs)
    echo "[INFO] Loaded environment variables from .env"
else
    echo "[ERROR] .env file not found! Please create one based on .env.example"
    exit 1
fi

# -----------------------
# Configuration Variables
# -----------------------

# Remote server credentials from .env
REMOTE_USER="$REMOTE_USERNAME"
REMOTE_HOST="$REMOTE_HOST"
REMOTE_PORT="$REMOTE_PORT"
REMOTE_SSH_KEY="$REMOTE_SSH_KEY"

# Paths
LOCAL_REPO_DIR="$(pwd)"
REMOTE_DEPLOY_DIR="/home/${REMOTE_USER}/PhotoPi/remote_server"  # Desired deployment directory on the remote server

# Log file with timestamp
LOG_FILE="deploy_to_remote_server_$(date +%Y%m%d_%H%M%S).log"

# Redirect all output to the log file and display in terminal
exec > >(tee -i $LOG_FILE)
exec 2>&1

# -----------------------
# Functions
# -----------------------

# Function to display informational messages
function echo_info() {
    echo -e "\e[32m[INFO]\e[0m $1"
}

# Function to display error messages
function echo_error() {
    echo -e "\e[31m[ERROR]\e[0m $1"
}

# Function to check if required commands are available
function check_prerequisites() {
    for cmd in rsync ssh; do
        if ! command -v $cmd &> /dev/null; then
            echo_error "$cmd is not installed. Please install it and retry."
            exit 1
        fi
    done
}

# Function to deploy files using rsync
function sync_files() {
    echo_info "Synchronizing remote_server/ to ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DEPLOY_DIR}/"

    rsync -avz --delete \
        -e "ssh -i ${REMOTE_SSH_KEY} -p ${REMOTE_PORT}" \
        --exclude 'venv/' \
        --exclude '__pycache__/' \
        --exclude '*.pyc' \
        --exclude '*.log' \
        remote_server/ "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DEPLOY_DIR}/"

    if [ $? -ne 0 ]; then
        echo_error "File synchronization failed."
        exit 1
    fi

    echo_info "File synchronization completed successfully."
}

# Function to set up remote environment
function setup_remote_env() {
    echo_info "Setting up Python virtual environment on remote server."

    ssh -i "${REMOTE_SSH_KEY}" -p "${REMOTE_PORT}" "${REMOTE_USER}@${REMOTE_HOST}" bash << EOF
        # Navigate to the deployment directory
        cd "${REMOTE_DEPLOY_DIR}"

        # Update package lists
        sudo apt-get update

        # Install Python3 and pip if not already installed
        sudo apt-get install -y python3 python3-venv python3-pip

        # Create a virtual environment if it doesn't exist
        if [ ! -d "venv" ]; then
            python3 -m venv venv
            echo "Virtual environment created."
        else
            echo "Virtual environment already exists."
        fi

        # Activate the virtual environment
        source venv/bin/activate

        # Upgrade pip
        pip install --upgrade pip

        # Install required Python packages
        pip install -r requirements.txt

        echo "Python environment setup completed."
EOF

    if [ $? -ne 0 ]; then
        echo_error "Remote environment setup failed."
        exit 1
    fi

    echo_info "Remote environment setup completed successfully."
}

# Function to optionally run build.py on remote server
function run_build_script() {
    read -p "Do you want to run build.py on the remote server now? (y/n): " choice
    case "$choice" in 
      y|Y ) 
        echo_info "Running build.py on remote server."
        ssh -i "${REMOTE_SSH_KEY}" -p "${REMOTE_PORT}" "${REMOTE_USER}@${REMOTE_HOST}" bash << EOF
            cd "${REMOTE_DEPLOY_DIR}"
            source venv/bin/activate
            python build.py --config config.yaml
EOF
        if [ $? -ne 0 ]; then
            echo_error "build.py execution failed on remote server."
            exit 1
        fi
        echo_info "build.py executed successfully on remote server."
        ;;
      n|N ) 
        echo_info "Skipping build.py execution."
        ;;
      * ) 
        echo_error "Invalid input. Skipping build.py execution."
        ;;
    esac
}

# -----------------------
# Main Script Execution
# -----------------------

# Check prerequisites
check_prerequisites

# Sync files to remote server
sync_files

# Set up remote Python environment
setup_remote_env

# Optionally run build.py
run_build_script

echo_info "Deployment to remote server completed successfully."
