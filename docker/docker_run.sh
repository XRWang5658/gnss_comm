#!/bin/bash

# This script is designed to run from within a package's sub-directory.
# It intelligently finds the catkin workspace root to mount the correct volumes.

# --- Configuration ---
IMAGE_NAME="gnss_comm-env"

# --- Path Calculation ---
# Get the directory where this script is located (POSIX-compliant)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Navigate up to the catkin 'src' directory (which contains all packages)
# Script is in .../src/gnss_comm/docker, so 'src' is two levels up.
CATKIN_SRC_DIR=$(realpath "${SCRIPT_DIR}/../..")

# Navigate up to the workspace root (which contains the 'src' directory)
# The workspace root is one level up from the 'src' directory.
WORKSPACE_ROOT=$(realpath "${CATKIN_SRC_DIR}/..")

# --- Build Step ---
echo ">>> Building Docker image '$IMAGE_NAME'..."
# The Dockerfile is in the same directory as the script
docker build -t $IMAGE_NAME "$SCRIPT_DIR"

# Exit if the build fails
if [ $? -ne 0 ]; then
    echo "Docker build failed. Aborting."
    exit 1
fi
echo ">>> Build successful."
echo

# --- Run Step ---
# Note: For GUI apps, you may need to run 'xhost +local:docker' on your host first.
echo ">>> Launching container..."
echo ">>> Workspace Root: ${WORKSPACE_ROOT}"
echo ">>> Mounting Catkin SRC: ${CATKIN_SRC_DIR} -> /root/catkin_ws/src"
echo ">>> Mounting Data folder: ${WORKSPACE_ROOT}/data -> /root/data"

docker run -it --rm \
  --gpus all \
  --net=host \
  -v "${CATKIN_SRC_DIR}":/root/catkin_ws/src \
  -v "${WORKSPACE_ROOT}/data":/root/data \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  $IMAGE_NAME \
  "${@:-bash}"