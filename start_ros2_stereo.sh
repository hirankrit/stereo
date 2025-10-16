#!/bin/bash
# Start ROS2 Stereo Camera Container (using jetson-containers)
# This script uses official NVIDIA jetson-containers with full Argus support

set -e

# Configuration
IMAGE="dustynv/ros:humble-ros-base-l4t-r36.2.0"
CONTAINER_NAME="ros2_stereo_camera"
WORKSPACE="/home/jay/projects/stereo_camera"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}ROS2 Stereo Camera Container${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# Check if container is already running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo -e "${GREEN}Container is already running!${NC}"
    echo -e "${YELLOW}Attaching to container...${NC}"
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

# Check if container exists but is stopped
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo -e "${YELLOW}Starting existing container...${NC}"
    docker start -ai $CONTAINER_NAME
    exit 0
fi

echo -e "${YELLOW}Starting new container...${NC}"
echo -e "Image: ${IMAGE}"
echo -e "Workspace: ${WORKSPACE}"
echo ""

# Run container with full Jetson support
docker run -it \
    --name $CONTAINER_NAME \
    --runtime nvidia \
    --network host \
    --privileged \
    --shm-size=8g \
    --volume /tmp/argus_socket:/tmp/argus_socket \
    --volume /etc/enctune.conf:/etc/enctune.conf \
    --volume /etc/nv_tegra_release:/etc/nv_tegra_release \
    --volume /tmp/nv_jetson_model:/tmp/nv_jetson_model \
    --volume $WORKSPACE:/workspace \
    --volume /dev:/dev \
    --workdir /workspace \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID=0 \
    $IMAGE \
    /bin/bash

echo -e "${GREEN}Container stopped.${NC}"
